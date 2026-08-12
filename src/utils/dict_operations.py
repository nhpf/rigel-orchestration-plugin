import copy
from typing import Any


# This function is really nice to have, because it can overwrite default values
# with the k8s kwargs passed in the Rigelfile
def _is_named_mapping_list(value: list[Any]) -> bool:
    return bool(value) and all(isinstance(item, dict) and "name" in item for item in value)


def _merge_named_lists(base: list[dict[str, Any]], overrides: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Merge Kubernetes-style lists (containers, env, volumes) by their name."""
    merged = copy.deepcopy(base)
    positions = {item["name"]: index for index, item in enumerate(merged)}
    for item in overrides:
        name = item["name"]
        if name in positions:
            index = positions[name]
            merged[index] = deep_merge(merged[index], item)
        else:
            positions[name] = len(merged)
            merged.append(copy.deepcopy(item))
    return merged


def deep_merge(base: dict[str, Any], overrides: dict[str, Any]) -> dict[str, Any]:
    """Deep-merge two dictionaries.

    If a key exists in both, and both values are dicts, recurse.
    Otherwise, override the base key with the overrides key.
    """
    merged = copy.deepcopy(base)
    for k, v in overrides.items():
        if k in merged and isinstance(merged[k], dict) and isinstance(v, dict):
            merged[k] = deep_merge(merged[k], v)
        elif (
            k in merged
            and isinstance(merged[k], list)
            and isinstance(v, list)
            and _is_named_mapping_list(merged[k])
            and _is_named_mapping_list(v)
        ):
            merged[k] = _merge_named_lists(merged[k], v)
        else:
            merged[k] = copy.deepcopy(v)
    return merged
