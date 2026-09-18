# Cross-module consistency tests

This suite links Buddy 1 telemetry and the shared mission controller only in
test code. It confirms that states and navigation actions cross the boundary
without duplicate translation tables or incompatible enum meanings.

```bash
make -C tests/integration test
make -C tests/integration sanitize
make -C tests/integration analyze
```
