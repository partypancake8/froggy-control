# Public-sharing review

Reviewed October 26, 2026. Scope: tracked source, documentation, schematic, four bench CSVs, bundled Debian packages, and published Git history. This is a publication/privacy review, not certification of the hardware or a comprehensive security audit.

- Gitleaks scans cover the working tree and all published commits. No credential findings remained at publication.
- Removed fixed workstation/campus and hotspot addresses, personal login defaults, private source-directory paths and commercial melody transcriptions from the published root commit. Commit metadata uses a GitHub noreply address. The original import is preserved privately; a fresh repository contains only the reviewed history because rewriting a branch alone does not remove GitHub's old commit objects.
- Pi account selection now uses `FROGGY_PI_USER` (default `froggy`). Existing Pi cards need their actual login supplied; no Pi is modified by this change. Campus login is prompted for or supplied through `FROGGY_UNIQNAME`.
- Main-Pi phone-home has no committed destination. It sends only when `FROGGY_MAC_HOST` is supplied during provisioning. The Zero path continues to select its destination at runtime.
- Reviewed schematic contents and PNG metadata; the CSVs contain motor telemetry fields rather than network or account information. Third-party package notices are preserved and described in `THIRD-PARTY.md`.
- Generated keys, profiles, boot files, discovery files and flash logs are ignored. Future files still need review before committing.

Validation uses controller/API tests, shell syntax checks, mocked provisioning and SSH checks, package checksums, and secret scanning. No motor commands, Pi deployments, card writes or live network changes are part of the review.

Local results: 33 tests passed, shell and Python syntax checks passed, all four offline package checksums matched, and Gitleaks 8.30.1 reported zero findings. Publication cannot guarantee that every future change is free of secrets.

The motor UI defaults to loopback with SSH tunneling. Camera transport and phone-home discovery are bench tools for a trusted network, not authenticated internet services. No change here commissions the Hall sensor, encoder capture, loaded stopping behavior or Pi-to-Pi protocol.
