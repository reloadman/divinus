#pragma once

// Acquire a "single instance" lock for the process.
//
// Implementation uses a pidfile + flock(2) and keeps the fd open for the lifetime
// of the process (lock is released automatically on exit).
//
// Returns 0 on success, -1 on failure (errno is set).
int single_instance_acquire(const char *name);

// Returns the pidfile path last attempted/used by this process,
// or NULL if none was attempted yet.
const char *single_instance_pidfile_path(void);


