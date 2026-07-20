# Running GitHub Actions Locally with `act`

Push, wait for GitHub, watch it fail, fix, push again — that loop is slow.
[`act`](https://github.com/nektos/act) runs your **existing** GitHub Actions
workflows on your own machine inside Docker, using the **same
`.github/workflows/*.yml` files** — no extra config, no duplicated scripts.
Get it green locally, *then* push.

> This is the plain, standard tool most developers use. There is nothing to
> customize: `act` reads the workflow you already have.

---

## 1. Prerequisite: Docker

`act` runs each job in a container, so Docker must be installed and running:

```bash
docker --version      # must succeed
docker ps             # daemon must be up
```

If Docker isn't installed, see https://docs.docker.com/engine/install/.

---

## 2. Install `act`

```bash
# One-line install script (Linux/macOS) → installs to a path you give
curl -sSL https://raw.githubusercontent.com/nektos/act/master/install.sh | bash -s -- -b ~/.local/bin

# or, if you already have the GitHub CLI:
gh extension install nektos/gh-act      # then run it as:  gh act ...

# or a package manager:
brew install act        # macOS / Linuxbrew    (Arch: sudo pacman -S act)
```

Verify:

```bash
act --version
```

On the **first** run `act` asks which runner image to use — pick **Medium**
(the standard `catthehacker/ubuntu` image). The choice is saved to `~/.actrc`.

---

## 3. Everyday commands

```bash
act -l                 # list the jobs act found in .github/workflows
act -n                 # dry run: print the plan, execute nothing
act -j build           # run ONE job by id (recommended)
act pull_request       # simulate a pull_request event instead of push
act -v                 # verbose, when something misbehaves
```

### For THIS repo

The CI here (`build.yml`, job id **`build`**, runner `ubuntu-22.04`) installs
system deps, bootstraps **vcpkg**, then configures and builds with CMake +
Ninja:

```bash
act -j build
```

> ⚠️ **Heads-up: this build is heavy.** vcpkg compiles OpenCV, GTK3, Ceres,
> Arrow and friends from source — the first run can take **hours** and needs
> several GB of disk. `act` runs against your local working tree (the `vcpkg`
> submodule must be checked out). For a quick check you usually don't need the
> whole build — see the next section.

---

## 4. The workflow: green locally → push

```bash
act -j build           # 1. run CI locally
# ...fix anything that fails, repeat until green...
git add -A && git commit -m "..."
git push               # 2. push only once it passed locally
```

---

## 5. Not every red build is *your* fault — read the log first

A failing CI run isn't always a bug in your code. Before "fixing" anything,
look at **which step failed and why**. You can read the real GitHub logs
without re-running:

```bash
gh run list --limit 5                 # see recent runs and their status
gh run view <run-id>                  # which step failed (look for the ✗)
gh run view <run-id> --log-failed     # the failing step's log
```

### Real example from this repo

A run here failed at the **Configure CMake** step with:

```
Downloading https://gitlab.freedesktop.org/freetype/.../freetype-VER-2-13-3.tar.gz
warning: HTTP error. Will retry... (3 retries)
error: curl: (22) The requested URL returned error: 502
error: building freetype:x64-linux-release failed with: BUILD_FAILED
```

`freetype` is a **transitive dependency** (GTK3 → cairo/pango → freetype), and
`gitlab.freedesktop.org` was returning **502/504** gateway errors. That's an
**upstream download outage**, not a bug in this repository. The fix is simply
to **re-run** once the host recovers:

```bash
# quick check whether the upstream is back:
curl -sI https://gitlab.freedesktop.org/freetype/freetype/-/archive/VER-2-13-3/freetype-VER-2-13-3.tar.gz

gh run rerun <run-id>        # re-run the failed run, or just push again
```

**Why it then stays green:** this workflow caches `build/vcpkg_installed`
(keyed on `vcpkg.json`). Once **one** run succeeds, the cache is populated and
later runs restore it instead of re-downloading — so the flaky freedesktop
mirror stops mattering. The trap is that the cache can't be populated until the
first run gets past the download, hence the occasional re-run.

**Durable fix (optional)** if freedesktop flakiness keeps biting: enable vcpkg
**asset caching** so downloaded tarballs are cached in GitHub Actions, e.g.
`X_VCPKG_ASSET_SOURCES=x-gha,readwrite` plus the `actions/cache` GHA env — then
even the *first* download is served from cache on retry.

---

## 6. Good to know (limitations)

- **`act` ≈ GitHub, not identical.** It uses `catthehacker` images, not
  GitHub's exact runner image. Close enough to catch the vast majority of
  failures; for maximum parity choose the **Large** image.
- **Linux jobs only.** `act` cannot run `windows-latest` / `macos-latest` jobs.
- **First run is slow** — it pulls the runner image and (here) builds all vcpkg
  dependencies. Later runs reuse caches and are much faster.
- **Network-dependent failures** (like the freetype example above) will
  reproduce under `act` too — because the download is genuinely failing, not
  the build logic.

---

## TL;DR

```bash
docker ps                        # Docker running?
act -l                           # what jobs exist?
gh run view <id> --log-failed    # if CI is red, read WHY first
act -j build                     # run CI locally → green? then git push
```
