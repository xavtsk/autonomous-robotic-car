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

INTEGRATION_PREDICATE_FIELDS = {
    'pending': 'b_has_command',
    'stop': 'b_has_stop',
    'status_ready': 'b_has_status',
    'reset_pending': 'b_has_reset',
    'ir_ready': 'b_has_ir_debug',
    'control_ready': 'b_has_control_debug',
    'reset_waiting': 'b_waits_for_reset',
    'reset_remote': 'b_is_remote_reset',
    'collecting': 'b_is_collecting',
    'control_seen': 'b_has_control_tick',
}


def integration_storage_aliases(subject):
    """Select the reviewed private layouts and reject other field bindings."""
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
    predicate_fields = {new: old for old, new in INTEGRATION_PREDICATE_FIELDS.items()}
    has_named_predicates = any(f'{field} : ' in joined for field in predicate_fields)
    if has_named_predicates:
        if not all(f'{field} : 1' in joined for field in predicate_fields):
            raise ValueError('Harness changed; review compact predicate fields')
        # Validate the same exact layout after translating the reviewed names.
        text = TOKEN.sub(lambda token: predicate_fields.get(token[0], token[0])
                         if token.lastgroup == 'identifier' else token[0], text)
        tokens = [match[0] for match in TOKEN.finditer(text) if match.lastgroup != 'comment']
        joined = ' '.join(tokens)
    sampling_fields = '''{
        uint8_t sensor : 2; uint8_t label : 2;
        bool reset_waiting : 1; bool reset_remote : 1; bool collecting : 1;
        #if APP_IR_DEBUG
        bool control_seen : 1;
        #endif
        }'''
    shared_fields = '''{
        bool pending : 1, stop : 1, status_ready : 1, reset_pending : 1;
        #if APP_IR_DEBUG
        bool ir_ready : 1, control_ready : 1;
        #endif
        }'''
    if 'bool pending : 1 ;' in joined:
        # The cleaned layout repeats the same type for each individual field.
        shared_fields = shared_fields.replace(': 1, ', ': 1; bool ')
    named_layout = 's3_sampling_flags_t' in tokens
    for fields, type_name, object_name in (
        (sampling_fields, 's3_sampling_flags_t', 'g_sampling'),
        (shared_fields, 's3_shared_flags_t', 'g_shared'),
    ):
        if named_layout:
            replace_once(text, f'typedef struct {fields} {type_name};', '')
            replace_once(text, f'static {type_name} {object_name};', '')
        else:
            replace_once(text, f'static struct {fields} {object_name};', '')
    for name, expression in INTEGRATION_STORAGE_NAMES.items():
        if name in tokens or expression.replace('.', ' . ') not in joined:
            raise ValueError('Unrecognized compact integration storage member: '+name)
    aliases = INTEGRATION_STORAGE_NAMES.copy()
    if has_named_predicates:
        for name, expression in aliases.items():
            owner, field = expression.split('.')
            aliases[name] = owner + '.' + INTEGRATION_PREDICATE_FIELDS.get(field, field)
    return aliases


def integration_storage_flags(subject):
    """Enable the test-only post-include adapter for the reviewed compact layout."""
    aliases = integration_storage_aliases(subject)
    if not aliases:
        return []
    flags = ['-DTEST_S3_COMPACT_STORAGE=1']
    if aliases['g_reset_waiting'] == 'g_sampling.reset_waiting':
        flags.append('-DTEST_S3_LEGACY_PREDICATES=1')
    return flags


def remote_request_flags(subject):
    """Select the request identifier spelling for the fixed comparison."""
    header = (Path(subject)/'app_program/controller/remote_control.h').read_text()
    fields = [token[0] for token in TOKEN.finditer(header)
              if token.lastgroup != 'comment']
    if 'uint32_t id ;' in ' '.join(fields):
        return ['-DTEST_REMOTE_ID_MEMBER=id']
    return []


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
    source = (candidate/'testing/integration/runtime_main.c').read_text()
    observer = Path(__file__).with_name('traces').joinpath('cycle_observer.h').read_text()
    declaration = 'static void cycle(void);'
    source = replace_once(source, declaration, observer + '\n' + declaration)
    source = replace_once(source,
        's3_integration_runtime_test_cycle(); } assert(!g_locks); }',
        's3_integration_runtime_test_cycle();\n    }\n    assert(!g_locks);\n    trace_runtime("cycle");\n}')
    receive_call = 'remote_control_receive(request_packet,sizeof request_packet);'
    source = replace_once(source, receive_call, receive_call + ' trace_runtime("receive");')
    reply_check = 'assert((reply_packet[1]&0x80U)==0U);'
    reply_trace = r'''
    printf("REPLY ");
    for (unsigned trace_i = 0; trace_i < sizeof reply_packet; ++trace_i)
    {
        printf("%02x", reply_packet[trace_i]);
    }
    putchar('\n');
    trace_runtime("take");'''
    source = replace_once(source, reply_check, reply_check + reply_trace)
    # Bind headers to the subject that supplies the independently linked owner.
    source = source.replace('"../../app_program/', '"' + (subject/'app_program').as_posix() + '/')
    destination.write_text(source, encoding='utf-8')
    return destination


def mqtt_source(candidate, destination):
    source = (candidate/'testing/subsystem1/host_main.c').read_text()
    observer = Path(__file__).with_name('traces').joinpath('mqtt_observer.h').read_text()
    callbacks = {
        'gp_incoming': 'trace_incoming',
        'gp_fragment': 'trace_fragment',
    }

    def observe_delivery(token):
        name = token[0]
        if name in callbacks and source[token.end():].lstrip().startswith('('):
            return callbacks[name]
        return name

    # Redirect fixture calls before adding the observers. Each observer
    # delegates once to the original registered callback.
    source = TOKEN.sub(observe_delivery, source)
    source = replace_once(source, 'u32_t sys_now(void)', observer + '\nu32_t sys_now(void)')
    publish_guard = 'if (g_publish_result != ERR_OK) { return g_publish_result; }'
    source = replace_once(source, publish_guard,
        'trace_publish(p_topic,p_payload,len,qos,retain);\n    ' + publish_guard)
    destination.write_text(source, encoding='utf-8')
    return destination


def operator_port_flags(subject):
    """Select the observer for the subject's two operator callbacks."""
    header = (subject/'app_program/controller/op_logic.h').read_text()
    tokens = ' '.join(m[0] for m in TOKEN.finditer(header) if m.lastgroup != 'comment')
    declaration = ('typedef struct { bool ( * %ssubmit ) ( const %s * '
                   'p_request ) ; int ( * %sprint ) ( const unsigned char * p_format '
                   ', ... ) ; } ')
    for prefix, flags in (('p_', []), ('', ['-DOP_TRACE_LEGACY_PORT=1'])):
        matches = sum(
            tokens.count((declaration % (prefix, request, prefix)) + type_name + ' ;')
            for type_name, request in (
                ('op_port_t', 's3_control_request_t'),
                ('op_logic_port_t', 's3_control_request_t'),
                ('op_logic_port_t', 's3_integration_request_t'),
            )
        )
        if matches == 1:
            return flags
    raise ValueError('Unrecognized operator callback declaration in ' + str(subject))


def operator_source(candidate, subject, destination):
    """Observe the existing whole GP20/GP21 fixture without changing assertions."""
    source = (candidate/'testing/integration/operator_main.c').read_text()
    source = replace_once(source, 'static void period(void) {',
        'static void period(void) {\n    const size_t trace_console_begin = g_console_length;\n')
    anchor = '''op_logic_step(&g_operator, now_ms(), go_press, op_logic_button_held(&g_stop_button),
                  op_logic_button_held(&g_go_button), s3_integration_status(&g_operator_status) ? &g_operator_status : 0);'''
    observation = '''
    const op_trace_context_t trace_context = {
        g_clock_us, g_remote_stop_seen, g_sleeps, g_locks, g_starts_requested,
        g_stops, gb_is_go_down, gb_is_stop_down, {g_raw[0], g_raw[1], g_raw[2]}};
    op_trace_write(&trace_context, &g_operator, &g_go_button, &g_stop_button, &g_operator_status, g_console,
                   trace_console_begin, g_console_length);
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
