"""Add observations to shared existing tests, without changing assertions."""
from pathlib import Path


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Harness changed; review instrumentation anchor: ' + old[:100])
    return text.replace(old, new, 1)


def runtime_source(candidate, subject, destination):
    source = (candidate/'testing/integration/runtime_test.c').read_text()
    observer = Path(__file__).with_name('traces').joinpath('runtime_observer.h').read_text()
    source = replace_once(source, 'static void cycle(void)', observer + '\nstatic void cycle(void)')
    source = replace_once(source,
        'sensor_sample_task(0,NULL);assert(!locked); }',
        'sensor_sample_task(0,NULL);assert(!locked); trace_runtime("cycle"); }')
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
