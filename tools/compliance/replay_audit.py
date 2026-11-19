#!/usr/bin/env python3
"""Collect every original replay assertion without turning failures into passes.

This diagnostic adapter leaves the tracked suite and its expectations intact.
It changes only fail-fast reporting to accumulated reporting in an in-memory
AST, so a pre-existing failure cannot conceal the rest of the fixture matrix.
The ordinary unmodified suite remains a required, separate failing/passing gate.
"""
import argparse
import ast
import contextlib
import hashlib
import inspect
import json
from pathlib import Path
import subprocess
import sys
import types


class CollectAssertions(ast.NodeTransformer):
    """Preserve each predicate/message; record false predicates as failures."""

    def visit_Assert(self, node):
        return ast.copy_location(ast.Expr(ast.Call(
            func=ast.Name(id='_record_assertion', ctx=ast.Load()),
            args=[node.test, ast.Lambda(args=ast.arguments(
                posonlyargs=[], args=[], kwonlyargs=[], kw_defaults=[],
                defaults=[]), body=node.msg or ast.Constant(None)),
                  ast.Constant(node.lineno)], keywords=[])), node)


def audit(suite, executable, output, paired_executable=None):
    """Run the unchanged expectation predicates and retain every replay output."""
    suite, executable, output = map(lambda value: Path(value).resolve(),
                                    (suite, executable, output))
    output.mkdir(parents=True, exist_ok=False)
    source = suite.read_bytes()
    report = dict(suite=str(suite), suite_sha256=hashlib.sha256(source).hexdigest(),
                  collection_mode='diagnostic accumulated assertions; no waived failures',
                  assertions=0, cases=0, failures=[], mismatches=0)
    module = types.ModuleType('compliance_replay_audit')
    module.__file__ = str(suite)

    def record_assertion(condition, message, line):
        report['assertions'] += 1
        if not condition:
            report['failures'].append(dict(line=line, message=message(),
                                           case=report['cases']))

    module._record_assertion = record_assertion
    tree = ast.fix_missing_locations(CollectAssertions().visit(
        ast.parse(source, filename=str(suite))))
    exec(compile(tree, str(suite), 'exec'), module.__dict__)
    with (output / 'cases.jsonl').open('w', encoding='utf-8') as evidence:
        def run_case(command, **kwargs):
            context = inspect.currentframe().f_back.f_locals
            kwargs['timeout'] = 30
            observed = subprocess.run([str(executable), *command[1:]], **kwargs)
            report['cases'] += 1
            row = dict(case=report['cases'],
                       inputs={key: context[key] for key in
                               ('name', 'raw', 'fraction', 'reverse', 'lead', 'trail')
                               if key in context},
                       input_sha256=hashlib.sha256(kwargs['input'].encode()).hexdigest(),
                       returncode=observed.returncode, stdout=observed.stdout,
                       stderr=observed.stderr)
            if paired_executable is not None:
                paired = subprocess.run([str(Path(paired_executable).resolve()),
                                         *command[1:]], **kwargs)
                row['paired'] = dict(returncode=paired.returncode,
                                     stdout=paired.stdout, stderr=paired.stderr)
                row['matched'] = ((observed.returncode, observed.stdout, observed.stderr)
                                  == (paired.returncode, paired.stdout, paired.stderr))
                report['mismatches'] += not row['matched']
            evidence.write(json.dumps(row) + '\n')
            return observed

        module.subprocess = types.SimpleNamespace(run=run_case)
        saved_argv = sys.argv
        try:
            sys.argv = [str(suite), str(executable)]
            # The original suite's final PASS string assumes fail-fast asserts.
            # Retain it as raw output, never reuse it as this audit's verdict.
            with (output / 'original-suite-console.log').open('w', encoding='utf-8') as console:
                with contextlib.redirect_stdout(console):
                    module.main()
        except Exception as error:
            report['execution_error'] = f'{type(error).__name__}: {error}'
        finally:
            sys.argv = saved_argv
    report['expectations_passed'] = not (report['failures'] or report.get('execution_error'))
    report['passed'] = report['expectations_passed'] and report['mismatches'] == 0
    (output / 'results.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', type=Path, required=True)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--paired-executable', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if sys.platform != 'linux':
        parser.error('Native replay execution requires Linux.')
    return not audit(args.suite, args.executable, args.output,
                     args.paired_executable)['passed']


if __name__ == '__main__':
    sys.exit(main())
