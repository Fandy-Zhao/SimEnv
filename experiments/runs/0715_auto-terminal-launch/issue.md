# Issue — Snap Code Prevents Dedicated Terminal Creation

## Goal

Ensure `auto.sh` opens separate GNOME Terminal windows for `junior_ctrl` and
FAST-LIO2 RViz when invoked from the Snap-packaged VS Code terminal.

## Evidence

The existing `gnome-terminal` wrapper exited with status 127:

```text
symbol lookup error: /snap/core20/.../libpthread.so.0:
undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
```

The direct `/usr/bin/gnome-terminal.real` binary succeeds when launched with a
minimal desktop environment containing only display, X authority, runtime, and
D-Bus variables.

## Scope

- Change only `auto.sh` terminal-launch environment.
- Do not change system ROS, system terminal installation, or Snap packages.
