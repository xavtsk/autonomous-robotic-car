"""Add observations to shared existing tests, without changing assertions."""
from pathlib import Path

try:
    from .format_owned import TOKEN
except ImportError:
    from format_owned import TOKEN


INTEGRATION_STORAGE_NAMES = {
    'g_reset_waiting': 'g_sampling.reset_waiting',
    'g_reset_remote': 'g_sampling.reset_remote',
    'g_collecting': 'g_sampling.collecting',
    'g_sensor': 'g_sampling.sensor',
    'g_label': 'g_sampling.label',
    'g_control_seen': 'g_sampling.control_seen',
    'g_ir_ready': 'g_shared.ir_ready',
    'g_control_ready': 'g_shared.control_ready',
}


def integration_storage_aliases(subject):
    """Select only the two reviewed private layouts; fail on unknown layouts."""
    text = (Path(subject)/'app_program/controller/s3_integration.c').read_text()
    tokens = [match[0] for match in TOKEN.finditer(text) if match.lastgroup != 'comment']
    joined = ' '.join(tokens)
    if 'g_sampling' not in tokens:
        if not all(name in tokens for name in INTEGRATION_STORAGE_NAMES):
            raise ValueError('Unrecognized original integration storage layout')
        for declaration in (
            'static bool g_reset_waiting;', 'static bool g_reset_remote;',
            'static bool g_collecting;', 'static bool g_ir_ready, g_ir_seen;',
            'static bool g_control_ready, g_control_seen;',
            'static s3_sensor_id_t g_sensor;', 'static ir_surface_t g_label;',
            '''static struct {
                unsigned pending : 1, stop : 1, status_ready : 1, reset_pending : 1;
            } g_shared;''',
        ):
            replace_once(text, declaration, '')
        return {}
    if joined.count('} g_sampling ;') != 1 or joined.count('} g_shared ;') != 1:
        raise ValueError('Unrecognized compact integration storage declarations')
    replace_once(text, '''static struct {
        uint8_t sensor : 2; uint8_t label : 2;
        bool reset_waiting : 1; bool reset_remote : 1; bool collecting : 1;
        #if APP_IR_DEBUG
        bool control_seen : 1;
        #endif
        } g_sampling;''', '')
    replace_once(text, '''static struct {
        bool pending : 1, stop : 1, status_ready : 1, reset_pending : 1;
        #if APP_IR_DEBUG
        bool ir_ready : 1, control_ready : 1;
        #endif
        } g_shared;''', '')
    for name, expression in INTEGRATION_STORAGE_NAMES.items():
        if name in tokens or expression.replace('.', ' . ') not in joined:
            raise ValueError('Unrecognized compact integration storage member: '+name)
    return dict(INTEGRATION_STORAGE_NAMES)


def integration_storage_flags(subject):
    """Enable the test-only post-include adapter for the reviewed compact layout."""
    return ['-DTEST_S3_COMPACT_STORAGE=1'] if integration_storage_aliases(subject) else []


def replace_once(text, old, new):
    """Replace one exact C-token sequence while ignoring formatting/comments.

    Strings remain literal tokens and multi-character operators remain atomic.
    An absent or repeated code anchor fails closed; an identical phrase inside
    a comment/string cannot manufacture a match. Only the copied harness changes.
    """
    anchor = [match[0] for match in TOKEN.finditer(old)
              if match.lastgroup != 'comment']
    source = [match for match in TOKEN.finditer(text)
              if match.lastgroup != 'comment']
    if not anchor:
        raise ValueError('Instrumentation anchor contains no C tokens')
    matches = [index for index in range(len(source) - len(anchor) + 1)
               if [item[0] for item in source[index:index + len(anchor)]] == anchor]
    if len(matches) != 1:
        raise ValueError('Harness changed; review instrumentation anchor: ' + old[:100])
    start = source[matches[0]].start()
    end = source[matches[0] + len(anchor) - 1].end()
    return text[:start] + new + text[end:]


def runtime_source(candidate, subject, destination):
    source = (candidate/'testing/integration/runtime_test.c').read_text()
    observer = Path(__file__).with_name('traces').joinpath('runtime_observer.h').read_text()
    source = replace_once(source, 'static void cycle(void)', observer + '\nstatic void cycle(void)')
    source = replace_once(source,
        'sensor_sample_task(0,NULL);assert(!locked); }',
        'sensor_sample_task(0, NULL);\n    assert(!locked);\n    trace_runtime("cycle");\n}')
    source = replace_once(source, 's3_remote_receive(p,sizeof p);',
                          's3_remote_receive(p,sizeof p); trace_runtime("receive");')
    source = replace_once(source, 'assert((p[1]&0x80U)==0U);',
        'assert((p[1]&0x80U)==0U);\n    printf("REPLY ");\n    for (unsigned trace_i=0; trace_i<sizeof p; ++trace_i) printf("%02x", p[trace_i]);\n    putchar(\'\\n\');\n    trace_runtime("take");')
    # The copied harness embeds the *subject* implementation, not the source
    # checkout that supplied test expectations. Only include locations change.
    source = source.replace('"../../app_program/', '"' + (subject/'app_program').as_posix() + '/')
    destination.write_text(source, encoding='utf-8')
    return destination


def mqtt_source(candidate, destination):
    source = (candidate/'testing/subsystem1/host_test.c').read_text()
    observer = Path(__file__).with_name('traces').joinpath('mqtt_observer.h').read_text()
    source = replace_once(source, 'u32_t sys_now(void)', observer + '\nu32_t sys_now(void)')
    source = replace_once(source, 'incoming = p; fragment = d;',
        'trace_original_incoming = p; trace_original_fragment = d; incoming = trace_incoming; fragment = trace_fragment;')
    source = replace_once(source, 'if (publish_result != ERR_OK) return publish_result;',
        'trace_publish(topic,payload,len,qos,retain);\n    if (publish_result != ERR_OK) return publish_result;')
    destination.write_text(source, encoding='utf-8')
    return destination


def operator_source(candidate, subject, destination):
    """Observe the existing whole GP20/GP21 fixture without changing assertions."""
    source = (candidate/'testing/integration/operator_test.c').read_text()
    source = replace_once(source, 'static void period(void) {',
        'static void period(void) {\n    const size_t trace_console_begin = console_len;\n')
    anchor = '''operator_step(&op, now_ms(), go, op_button_held(&stop_b),
                  op_button_held(&go_b), s3_integration_status(&st) ? &st : 0);'''
    observation = '''
    const op_trace_context_t trace_context = {
        clock_value, remote_stop_seen, sleeps, locked, starts_requested,
        stops, go_down, stop_down, {raw[0], raw[1], raw[2]}};
    op_trace_write(&trace_context, &op, &go_b, &stop_b, &st, console,
                   trace_console_begin, console_len);
'''
    source = replace_once(source, anchor, anchor + observation)
    observer = (Path(__file__).with_name('traces')/'op_trace.h').resolve().as_posix()
    include = '#include "../../app_program/controller/op_logic.h"'
    source = replace_once(source, include, include+'\n#include "'+observer+'"')
    source = source.replace('"../../app_program/', '"' + (subject/'app_program').as_posix() + '/')
    destination.write_text(source, encoding='utf-8')
    return destination


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Select reviewed host fixture compatibility flags.')
    parser.add_argument('--private-storage-flag', type=Path, required=True, metavar='SUBJECT')
    args = parser.parse_args()
    print(' '.join(integration_storage_flags(args.private_storage_flag.resolve())))
