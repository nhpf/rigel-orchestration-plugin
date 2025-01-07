import copy


# This function is really nice to have, because it can overwrite default values
# with the k8s kwargs passed in the Rigelfile
def deep_merge(base: dict, overrides: dict) -> dict:
    """Deep-merge two dictionaries.

    If a key exists in both, and both values are dicts, recurse.
    Otherwise, override the base key with the overrides key.
    """
    merged = copy.deepcopy(base)
    for k, v in overrides.items():
        if k in merged and isinstance(merged[k], dict) and isinstance(v, dict):
            merged[k] = deep_merge(merged[k], v)
        else:
            merged[k] = v
    return merged
