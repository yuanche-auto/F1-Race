# Development and review workflow

* One branch per layer: `layer/L0`, `layer/L1`, ...
* When a layer is complete, the builder sends a **review request** to an
  independent reviewer with: branch, commit hash, scope, reproduction commands
  and the claims to verify. The builder then starts the next layer.
* The reviewer works read-only on the given commit and answers with a verdict
  **APPROVE** or **CHANGES_REQUESTED**; every finding carries a severity
  (blocker / major / minor / nit) and evidence (file:line, command, numbers).
* Fixes are added as new commits on the same branch and sent back for
  re-review.
* Only an approved branch is merged into `main` and pushed. Nothing reaches
  `main` without review.
* Progress and the review log are kept in [STATUS.md](../STATUS.md).

Standing checks for every review:

1. no third-party course or proprietary material in the repository
   (`tests/test_no_course_content.py` plus manual inspection);
2. track data not committed, upstream commit and SHA-256 pinned
   (`scripts/fetch_tracks.py`);
3. reproduction commands run and `pytest` passes.
