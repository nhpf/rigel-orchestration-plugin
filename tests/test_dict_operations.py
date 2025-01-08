from src.utils.dict_operations import deep_merge


def test_deep_merge_basic() -> None:
    """Test deep_merge with basic dictionaries."""
    base = {"spec": {"containers": [{"name": "ros-app"}]}}
    overrides = {"spec": {"containers": [{"image": "custom"}]}}
    merged = deep_merge(base, overrides)
    # We expect the entire container list is replaced, so the final object is:
    assert merged["spec"]["containers"] == [{"image": "custom"}]


def test_deep_merge_nested() -> None:
    """Test deep_merge with nested dictionaries."""
    base = {"a": {"b": {"c": 1}, "d": 2}}
    overrides = {"a": {"b": {"c": 10, "x": 99}}}
    merged = deep_merge(base, overrides)
    assert merged["a"]["b"]["c"] == 10
    assert merged["a"]["d"] == 2
    assert merged["a"]["b"]["x"] == 99
