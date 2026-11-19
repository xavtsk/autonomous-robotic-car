"""Add observations to shared existing tests, without changing assertions."""
from pathlib import Path

try:
    from .format_owned import TOKEN
except ImportError:
    from format_owned import TOKEN


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
