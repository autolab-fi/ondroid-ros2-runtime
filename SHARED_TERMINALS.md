# Shared laboratory terminal transport

`ondroid_lab_terminal` is a small independent Python package hosted in this
repository. It has no ROS dependency. Networking pins its Git revision through
the `ondroid-lab-terminal` distribution; both runtimes use the same bounded PTY
implementation and terminal-process cleanup. The package does not authorize
users: callers must check ownership and serialize session lifecycle changes.

`Terminal(container, rcfile=...)` opens a shell inside an existing isolated
container. The rcfile is selected by trusted server code. Transport messages
retain the existing ROS protocol: base64 bytes, a 64 KiB output ring, 4 KiB input
limit, sequence acknowledgments, reconnect ownership and bounded resizing.
Closing a terminal stops its process tree; ending a workspace remains the
runtime's responsibility. No student directory or host network is mounted.

The extraction was verified with the existing real-container ROS workspace
acceptance suite: four terminals, shared files, user isolation, saved projects,
closing foreground and detached processes, closing/reopening the last tab, and
a passing sandbox exercise. Run unit tests with
`python -m unittest discover -s tests -p 'test_*.py'`.

The Docker service includes this module directly from its source revision.
External runtimes must pin an immutable revision and run their own container
acceptance tests before upgrading. Updating the library does not redeploy the
already qualified production ROS service automatically.
