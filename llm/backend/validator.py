"""
Backend validator - validates LLM output against whitelist and parameter rules.
Never trust LLM's own is_valid claim.
"""
from .schema import LLMOutput, ValidationResult, Intent


PRIMITIVE_INTENTS = {
    Intent.STATUS,
    Intent.READY,
    Intent.OPEN_GRIPPER,
    Intent.CLOSE_GRIPPER,
    Intent.MOVE_UP,
}

# Semantic tasks carry object/container labels and are routed to the
# visual_grasp bridge (/visual_grasp/task) instead of /stage2_arm/move.
SEMANTIC_INTENTS = {
    Intent.PICK,
    Intent.PLACE_AT,
    Intent.PLACE_INTO,
    Intent.CLEAR_TABLE,
}

ALLOWED_INTENTS = PRIMITIVE_INTENTS | SEMANTIC_INTENTS | {Intent.INVALID}

# Map intent to /stage2_arm/move service command
SERVICE_MAP = {
    Intent.STATUS: "status",
    Intent.READY: "ready",
    Intent.OPEN_GRIPPER: "open",
    Intent.CLOSE_GRIPPER: "close",
    Intent.MOVE_UP: "up",
}

# Map semantic intent to the visual_grasp bridge "task" field
# (multitask/bridge.py: SUPPORTED_TASKS = {pick, place_at, place_into, clear_table})
SEMANTIC_SERVICE_MAP = {
    Intent.PICK: "pick",
    Intent.PLACE_AT: "place_at",
    Intent.PLACE_INTO: "place_into",
    Intent.CLEAR_TABLE: "clear_table",
}

PRIMITIVE_SERVICE_NAME = "/stage2_arm/move"
SEMANTIC_SERVICE_NAME = "/visual_grasp/task"

# Intents that should NOT have parameters
NO_PARAM_INTENTS = {
    Intent.STATUS,
    Intent.READY,
    Intent.OPEN_GRIPPER,
    Intent.CLOSE_GRIPPER,
    Intent.INVALID,
}



def validate_llm_output(llm_output: LLMOutput) -> ValidationResult:
    """
    Validate LLM output against backend rules.
    Returns ValidationResult with passed=True only if all checks pass.
    """
    warnings = []

    # Check 1: Intent must be in whitelist
    if llm_output.intent not in ALLOWED_INTENTS:
        return ValidationResult(
            passed=False,
            error=f"Unknown intent: {llm_output.intent}",
            warnings=warnings
        )

    # Check 2: If is_valid=false, intent must be invalid
    if not llm_output.is_valid and llm_output.intent != Intent.INVALID:
        return ValidationResult(
            passed=False,
            error="is_valid=false but intent is not 'invalid'",
            warnings=warnings
        )

    # Check 3: Invalid intent should not proceed to execution
    if llm_output.intent == Intent.INVALID:
        return ValidationResult(
            passed=True,
            service_command=None,
            warnings=["Command marked as invalid by LLM, will not execute"]
        )

    # Check 4: Certain intents should not have parameters
    if llm_output.intent in NO_PARAM_INTENTS:
        if llm_output.parameters and len(llm_output.parameters) > 0:
            # Allow but warn - don't fail for extra params, just ignore them
            warnings.append(f"Intent {llm_output.intent.value} should not have parameters, ignoring")

    # Check 5: move_up parameter validation
    if llm_output.intent == Intent.MOVE_UP:
        distance = llm_output.parameters.get("distance_m", 0.05)

        if not isinstance(distance, (int, float)):
            return ValidationResult(
                passed=False,
                error="distance_m must be a number",
                warnings=warnings
            )

        if distance < 0.01:
            return ValidationResult(
                passed=False,
                error=f"distance_m={distance} is below minimum 0.01m",
                warnings=warnings
            )

        if distance > 0.10:
            return ValidationResult(
                passed=False,
                error=f"distance_m={distance} exceeds maximum 0.10m",
                warnings=warnings
            )

    # Check 6: semantic task parameter validation (mirrors visual_grasp's
    # multitask/bridge.py:_validate_task_args, kept in sync by hand since
    # this repo is a plain copy, not a live import of that module)
    if llm_output.intent in SEMANTIC_INTENTS:
        error = _validate_semantic_params(llm_output.intent, llm_output.parameters)
        if error:
            return ValidationResult(passed=False, error=error, warnings=warnings)

    # All checks passed
    if llm_output.intent in SEMANTIC_INTENTS:
        return ValidationResult(
            passed=True,
            service_name=SEMANTIC_SERVICE_NAME,
            service_command=SEMANTIC_SERVICE_MAP[llm_output.intent],
            warnings=warnings
        )

    return ValidationResult(
        passed=True,
        service_name=PRIMITIVE_SERVICE_NAME,
        service_command=SERVICE_MAP.get(llm_output.intent),
        warnings=warnings
    )


def _validate_semantic_params(intent: Intent, parameters: dict) -> str | None:
    """Return an error string if required object/container labels are missing."""
    source = parameters.get("source_label")
    container = parameters.get("container_label")
    xyz = parameters.get("target_xyz")
    labels = parameters.get("labels")

    if intent in (Intent.PICK, Intent.PLACE_AT, Intent.PLACE_INTO) and not source:
        return f"{intent.value} requires parameters.source_label"
    if intent == Intent.PLACE_AT and not xyz:
        return "place_at requires parameters.target_xyz"
    if intent == Intent.PLACE_INTO and not container and not xyz:
        return "place_into requires parameters.container_label or parameters.target_xyz"
    if intent == Intent.CLEAR_TABLE and not labels:
        return "clear_table requires parameters.labels"
    return None
