from src.utils.dict_operations import deep_merge


def test_deep_merge_nested_mappings() -> None:
    base = {"a": {"b": {"c": 1}, "d": 2}}
    overrides = {"a": {"b": {"c": 10, "x": 99}}}

    merged = deep_merge(base, overrides)

    assert merged == {"a": {"b": {"c": 10, "x": 99}, "d": 2}}
    assert base == {"a": {"b": {"c": 1}, "d": 2}}


def test_deep_merge_named_kubernetes_lists() -> None:
    base = {
        "containers": [
            {"name": "app", "image": "old", "env": [{"name": "PRESERVED", "value": "yes"}]},
        ],
    }
    overrides = {
        "containers": [
            {"name": "app", "image": "new", "env": [{"name": "ADDED", "value": "yes"}]},
            {"name": "sidecar", "image": "helper"},
        ],
    }

    merged = deep_merge(base, overrides)

    assert merged["containers"][0]["image"] == "new"
    assert {item["name"] for item in merged["containers"][0]["env"]} == {"PRESERVED", "ADDED"}
    assert merged["containers"][1] == {"name": "sidecar", "image": "helper"}


def test_deep_merge_replaces_unnamed_lists() -> None:
    assert deep_merge({"args": ["old"]}, {"args": ["new"]}) == {"args": ["new"]}
