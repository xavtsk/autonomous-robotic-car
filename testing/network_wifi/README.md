# Wi-Fi connection recovery tests

Run from the repository root:

```sh
make -C testing/network_wifi test sanitize
```

`policy_test.c` exercises the same `wifi_retry_policy.h` used by the radio
service. `runtime_test.c` runs the production `cyw43_utk_task()` with fake
CYW43/lwIP/MQTT APIs and a virtual clock. No hotspot, robot, real credentials or
real-time sleeps are involved. The fakes belong only to host tests; do not add
their include path to deployment firmware.

Defaults (compile-time overrides in microseconds):

- `WIFI_CONNECT_TIMEOUT_US`: 20,000,000. Total join-to-IP deadline; changing
  between DOWN, JOIN and NOIP does not extend it.
- `WIFI_REJOIN_DELAY_US`: 5,000,000. First retry delay.
- `WIFI_REJOIN_MAX_DELAY_US`: 30,000,000. Retries back off 5, 10, 20, then 30
  seconds, capped there until a link becomes UP. UP resets the delay to 5 s.

An accepted join request is not yet a connection. At the 20 s deadline the
single radio owner calls the official `cyw43_wifi_leave()` API, then continues
its 2 ms radio/lwIP service while waiting for the next retry. A negative link
status or rejected join request schedules a retry too, rather than parking
forever. Link DOWN after a successful UP gets the same recovery; loss of only
the IP address gets a fresh 20 s DHCP window. UP during a waiting backoff
cancels that retry. A true bus/interface initialization failure still parks at
the original low rate; retrying a broken bus is outside this change.

Coverage includes normal success, indefinite JOIN/NOIP, transitions that must
not reset the timeout, changing negative statuses, request rejection, leave
rejection, recovery after UP, capped backoff/reset, late UP, 32/64-bit clock
wrap, no stale reported IP while down, no lock held over task sleeps or
application callbacks, and continued radio/DHCP polling during retry delays.

These tests establish software policy and integration wiring, not actual
hotspot reliability or worst-case scheduling. The 20 s value is an initial
test setting. On hardware, test hotspot absent at boot then enabled, normal
connect, repeated loss/recovery, and GP21/motion timing while the connection
is retried. Do not assume a recovered network implies a recovered MQTT
session; MQTT retains its own reconnect state machine.
