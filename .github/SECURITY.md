# Security Policy

## Supported Versions

| Version | Supported |
|---------|-----------|
| Latest (`main`) | Yes |
| Tagged releases | Yes (latest tag only) |
| Older tags | No |

## Reporting a Vulnerability

**Please do not report security vulnerabilities through public GitHub Issues.**

If you discover a security vulnerability — for example, a remotely exploitable bug in the CAN/USB communication stack, an authentication bypass in the OTA bootloader, or an integer overflow in a safety-critical calculation — please report it privately:

1. Open a [GitHub Security Advisory](https://github.com/kitjesen/vectorfoc/security/advisories/new) (preferred).
2. Alternatively, email the maintainers directly (check the repository's GitHub profile for contact information).

Please include:
- A description of the vulnerability and its potential impact
- Steps to reproduce (firmware firmware_version, hardware board, test setup)
- Any proof-of-concept code or log output

We will acknowledge your report within **72 hours** and aim to release a fix within **14 days** for critical issues.

## Security Considerations for Embedded Deployment

VectorFOC is firmware for motor_runtime controllers. When deploying in a safety-critical or networked environment, consider:

- **CAN bus**: The firmware does not implement CAN bus authentication. All nodes on the same CAN network can send commands. Ensure physical access to the CAN bus is control.
- **OTA bootloader**: The bootloader does not verify firmware image signatures. Only deploy OTA updates over trusted channels.
- **USB CDC**: The USB debug interface exposes parameter read/write. Disable `HW_USB_ENABLED` in production deployments that do not require it.
- **Fault thresholds**: Review the protection thresholds in `src/safety/fault_definition.h` against the actual motor, board configuration, and application requirements.
