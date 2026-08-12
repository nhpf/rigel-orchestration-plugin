# Operator handover

This checklist covers the standalone `rigel-k8s` workflow. Complete it once per
target cluster and repeat the smoke test after material cluster or image changes.

## Repository controls

Protect `master` after the handover branch is merged. Require pull requests and
these `CI` checks:

- `quality`
- `unit (3.10)`
- `unit (3.11)`
- `unit (3.12)`
- `unit (3.13)`
- `integration`

Disallow force pushes and branch deletion. Decide whether administrators may
bypass these rules. Dependabot is configured for Python and GitHub Actions
updates; its pull requests should pass the same checks.

The repository has no selected software license. Choose one with the copyright
owner before distributing source or release artifacts outside the current
ownership boundary.

## Cluster prerequisites

Record the following for each environment:

- cluster and kubeconfig context;
- application and observability namespaces;
- namespaced RBAC for Deployments, Services, Pods, pod logs/exec, and PVCs;
- PV permissions if static `hostPath` volumes are permitted;
- approved StorageClass, capacity, reclaim policy, and backup policy;
- image registry, immutable image digest, and pull credentials;
- result source path, retention period, export destination, and data owner; and
- Helm version and approved observability chart versions when enabled.

The plugin does not create the application namespace or registry credentials.

## Acceptance run

Use a disposable namespace for the first run:

```bash
kubectl config current-context
kubectl auth can-i create deployments.apps --namespace <namespace>
kubectl auth can-i create persistentvolumeclaims --namespace <namespace>
poetry run rigel-k8s Rigelfile --dry-run
poetry run rigel-k8s Rigelfile
kubectl rollout status deployment/<deployment-name> --namespace <namespace>
poetry run rigel-k8s Rigelfile --collect-results ./results-<run-id>
poetry run rigel-k8s Rigelfile --cleanup
```

Confirm ROS behavior independently of Kubernetes readiness, verify the exported
files and checksums, and confirm the PVC remains after the default cleanup.

Only after a successful export and retention check should destructive cleanup be
tested:

```bash
poetry run rigel-k8s Rigelfile --cleanup --delete-storage --uninstall-observability
```

## Failure recovery

Inspect workload state and events:

```bash
kubectl get deployment,pod,service,pvc --namespace <namespace>
kubectl describe deployment/<deployment-name> --namespace <namespace>
kubectl get events --namespace <namespace> --sort-by=.lastTimestamp
kubectl logs deployment/<deployment-name> --namespace <namespace> --all-containers
```

If deployment fails, keep storage in place, correct the image or configuration,
and rerun the same job. Resource application is idempotent. If export fails,
verify that the pod is ready, `results.source_path` exists, the configured
container name is correct, and the image contains `tar`:

```bash
kubectl exec deployment/<deployment-name> --namespace <namespace> -- \
  test -e <results.source_path>
```

Do not use `--delete-storage` while results are awaiting export or recovery.

## Releases

Update the package version in `pyproject.toml`, merge with green CI, and create a
matching `v<version>` tag. The Release workflow verifies the version, builds the
wheel and source archive, and attaches both to a GitHub release. PyPI publishing
is intentionally not configured; adding it requires a project owner and a PyPI
trusted-publisher decision.
