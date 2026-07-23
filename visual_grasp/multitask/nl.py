"""Natural-language task parsing for the multitask executor.

This module is a local, rule-based stand-in for the planned LLM keyword
extraction step. It keeps the graduation-design architecture intact:
natural language -> extracted task/object/container keywords -> existing
TaskExecutor/task_library calls. No external LLM API is used, so parsing is
deterministic and easy to unit test.
"""
from dataclasses import asdict, dataclass
import re


@dataclass(frozen=True)
class ParsedTask:
    """Structured task shape accepted by TaskExecutor."""

    task: str
    source_label: str = None
    labels: tuple = ()
    container_label: str = None
    place_xyz: tuple = None
    policy: str = "nearest"
    text: str = ""

    def to_dict(self):
        return asdict(self)


class NLParseError(ValueError):
    """Raised when an utterance cannot be mapped to a supported task."""


# Canonical labels are YOLO/task labels used by config.yaml. Some natural
# language container words (bucket/bin/box/桶/箱子) are mapped to "bowl" because
# the current simulated task layer only has a reliable bowl container.
LABEL_ALIASES = {
    "cup": ("cup", "mug", "杯子", "杯", "水杯"),
    "bottle": ("bottle", "瓶子", "瓶", "瓶装物"),
    "bowl": (
        "bowl", "container", "bin", "bucket", "box",
        "碗", "容器", "桶", "盒子", "箱子",
    ),
}
DEFAULT_NL_CFG = {"objects": {"graspable": ["cup", "bottle"], "containers": ["bowl"]}}

PICK_WORDS = ("pick", "grab", "grasp", "take", "lift", "拿", "抓", "夹", "取")
PLACE_WORDS = ("put", "place", "move", "drop", "放", "放到", "放进", "移动")
INTO_WORDS = ("into", "inside", "in", "to", "里", "里面", "内", "进", "放进", "放到")
CLEAR_WORDS = ("clear table", "clean table", "clear", "clean", "清桌", "清理", "清空", "收拾")


def parse_nl(text, cfg=None, policy="nearest"):
    """Parse a short Chinese/English command into a ParsedTask.

    Supported tasks match the existing execution layer: pick, place_at,
    place_into, and clear_table.
    """
    cfg = cfg or _load_default_cfg()
    original = (text or "").strip()
    if not original:
        raise NLParseError("empty command")

    normalized = _normalize(original)
    mentions = _find_label_mentions(normalized)
    xyz = _find_xyz(normalized)

    if _has_any(normalized, CLEAR_WORDS):
        return _parse_clear_table(original, normalized, mentions, xyz, cfg, policy)

    container = _first_label(mentions, cfg["objects"].get("containers", ()))
    source = _first_label(mentions, cfg["objects"].get("graspable", ()))

    if _looks_like_place_into(normalized, container):
        if source is None:
            raise NLParseError("place_into requires a source object, e.g. 'put cup into bowl'")
        return ParsedTask("place_into", source_label=source, container_label=container,
                          policy=policy, text=original)

    if xyz is not None and _has_any(normalized, PLACE_WORDS):
        if source is None:
            raise NLParseError("place_at requires a source object, e.g. 'place cup at 0.4,0.1,0.17'")
        return ParsedTask("place_at", source_label=source, place_xyz=xyz,
                          policy=policy, text=original)

    if _has_any(normalized, PICK_WORDS):
        if source is None:
            raise NLParseError("pick requires a source object, e.g. 'pick cup'")
        return ParsedTask("pick", source_label=source, policy=policy, text=original)

    # Chinese commands like "把杯子放到碗里" may be easier to classify from labels.
    if source and container and _has_any(normalized, PLACE_WORDS + INTO_WORDS):
        return ParsedTask("place_into", source_label=source, container_label=container,
                          policy=policy, text=original)

    raise NLParseError(
        "could not parse command; supported examples: 'pick cup', "
        "'把杯子放到碗里', 'clear table into bowl'"
    )


def execute_parsed_task(parsed, executor):
    """Dispatch a ParsedTask to an existing TaskExecutor instance."""
    if parsed.task == "pick":
        return executor.pick(parsed.source_label, parsed.policy)
    if parsed.task == "place_at":
        return executor.place_at(parsed.source_label, parsed.place_xyz, parsed.policy)
    if parsed.task == "place_into":
        return executor.place_into(parsed.source_label, parsed.container_label, parsed.place_xyz,
                                   parsed.policy)
    if parsed.task == "clear_table":
        return executor.clear_table(list(parsed.labels), parsed.container_label, parsed.place_xyz,
                                    parsed.policy)
    raise NLParseError(f"unsupported parsed task: {parsed.task}")


def _parse_clear_table(original, normalized, mentions, xyz, cfg, policy):
    graspable = tuple(cfg["objects"].get("graspable", ()))
    containers = tuple(cfg["objects"].get("containers", ()))
    labels = tuple(dict.fromkeys(
        label for _, label in mentions if label in graspable
    )) or graspable
    if not labels:
        raise NLParseError("clear_table requires at least one configured graspable label")
    container = _first_label(mentions, containers)
    if container is None and containers:
        container = containers[0]
    return ParsedTask("clear_table", labels=labels, container_label=container,
                      place_xyz=xyz, policy=policy, text=original)


def _normalize(text):
    text = text.lower().strip()
    text = re.sub(r"[，。！？；：、]", " ", text)
    text = re.sub(r"[!?;:]", " ", text)
    return re.sub(r"\s+", " ", text)


def _find_label_mentions(text):
    mentions = []
    aliases = []
    for label, words in LABEL_ALIASES.items():
        aliases.extend((word, label) for word in words)
    for word, label in sorted(aliases, key=lambda item: len(item[0]), reverse=True):
        pattern = re.escape(word)
        if word.isascii() and word.replace("_", "").isalnum():
            pattern = rf"\b{pattern}\b"
        for match in re.finditer(pattern, text):
            mentions.append((match.start(), label))
    mentions.sort(key=lambda item: item[0])

    deduped = []
    seen = set()
    for pos, label in mentions:
        key = (pos, label)
        if key not in seen:
            deduped.append((pos, label))
            seen.add(key)
    return deduped


def _find_xyz(text):
    number = r"[-+]?\d+(?:\.\d+)?"
    match = re.search(rf"({number})\s*[, ]\s*({number})\s*[, ]\s*({number})", text)
    if not match:
        return None
    return tuple(float(match.group(i)) for i in range(1, 4))


def _first_label(mentions, allowed):
    allowed = set(allowed)
    for _, label in mentions:
        if label in allowed:
            return label
    return None


def _has_any(text, words):
    return any(word in text for word in words)


def _looks_like_place_into(text, container):
    return container is not None and _has_any(text, PLACE_WORDS) and _has_any(text, INTO_WORDS)


def _load_default_cfg():
    try:
        from .config import load_config
        return load_config()
    except ImportError:
        return DEFAULT_NL_CFG
