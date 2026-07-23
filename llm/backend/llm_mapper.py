"""
LLM Intent Mapper - converts natural language to structured JSON command.
Supports Anthropic Claude and OpenAI GPT.
"""
import json
import re
from typing import Optional
from .config import LLM_API_KEY, LLM_MODEL, LLM_PROVIDER
from .schema import LLMOutput, Intent


SYSTEM_PROMPT = """You are an intent mapper for a robotic arm demo.

You must convert the user's command into one JSON object only.
Do not output Markdown.
Do not wrap in code blocks.
Do not explain outside JSON.

The robot supports two kinds of intents.

Primitive intents (no object involved, routed to /stage2_arm/move):
- status: show current robot or joint status
- ready: move the arm to ready pose
- open_gripper: open the gripper
- close_gripper: close the gripper
- move_up: move the end effector slightly upward

Semantic intents (object manipulation, routed to /visual_grasp/task; only the
object labels the perception/sim system already knows about are meaningful,
e.g. cup, bottle, bowl):
- pick: pick up a named object. parameters: {"source_label": "<object>"}
- place_at: pick a named object and place it at explicit xyz coordinates.
  parameters: {"source_label": "<object>", "target_xyz": [x, y, z]}
- place_into: pick a named object and place it into a named container.
  parameters: {"source_label": "<object>", "container_label": "<container>"}
- clear_table: pick up a list of named objects one by one and place each into
  a named container. parameters: {"labels": ["<object>", ...], "container_label": "<container>"}

If the user command does not match any of these, return is_valid=false and intent="invalid".

For move_up, distance_m is optional. If the user does not specify a distance, use 0.05. The allowed range is 0.01 to 0.10.

For semantic intents, always fill in every parameter listed above for that intent — do not omit source_label/container_label/labels. If the command is ambiguous about which object or container, still pick the single most likely label mentioned in the command rather than returning invalid.

Return exactly this JSON structure:
{
  "schema_version": "1.0",
  "is_valid": boolean,
  "intent": string,
  "parameters": object,
  "confidence": number,
  "reason": string
}

Examples:
- "show me the robot status" -> {"schema_version": "1.0", "is_valid": true, "intent": "status", "parameters": {}, "confidence": 0.95, "reason": "User asked for robot status."}
- "go back to ready pose" -> {"schema_version": "1.0", "is_valid": true, "intent": "ready", "parameters": {}, "confidence": 0.92, "reason": "User wants arm to return to ready pose."}
- "open the gripper" -> {"schema_version": "1.0", "is_valid": true, "intent": "open_gripper", "parameters": {}, "confidence": 0.98, "reason": "User wants to open the gripper."}
- "grab it" -> {"schema_version": "1.0", "is_valid": true, "intent": "close_gripper", "parameters": {}, "confidence": 0.85, "reason": "User wants to close/grab, mapped to close_gripper."}
- "move up a little" -> {"schema_version": "1.0", "is_valid": true, "intent": "move_up", "parameters": {"distance_m": 0.05}, "confidence": 0.90, "reason": "User wants arm to move upward."}
- "move up 8 centimeters" -> {"schema_version": "1.0", "is_valid": true, "intent": "move_up", "parameters": {"distance_m": 0.08}, "confidence": 0.95, "reason": "User specified 8cm upward movement."}
- "pick up the bottle" -> {"schema_version": "1.0", "is_valid": true, "intent": "pick", "parameters": {"source_label": "bottle"}, "confidence": 0.9, "reason": "User wants to pick up the bottle."}
- "put the cup in the bowl" -> {"schema_version": "1.0", "is_valid": true, "intent": "place_into", "parameters": {"source_label": "cup", "container_label": "bowl"}, "confidence": 0.93, "reason": "User wants the cup placed into the bowl."}
- "把杯子放到碗里" -> {"schema_version": "1.0", "is_valid": true, "intent": "place_into", "parameters": {"source_label": "cup", "container_label": "bowl"}, "confidence": 0.93, "reason": "User wants the cup placed into the bowl."}
- "clear the table, put the cup and bottle in the bowl" -> {"schema_version": "1.0", "is_valid": true, "intent": "clear_table", "parameters": {"labels": ["cup", "bottle"], "container_label": "bowl"}, "confidence": 0.88, "reason": "User wants multiple objects cleared into the bowl."}
- "write me a poem" -> {"schema_version": "1.0", "is_valid": false, "intent": "invalid", "parameters": {}, "confidence": 0.0, "reason": "Not a robot command."}"""


def extract_json_from_response(text: str) -> Optional[dict]:
    """Extract JSON from LLM response, handling markdown code blocks."""
    # Try to find JSON in code blocks first
    code_block_pattern = r"```(?:json)?\s*([\s\S]*?)```"
    matches = re.findall(code_block_pattern, text)
    if matches:
        text = matches[0]

    # Clean up the text
    text = text.strip()

    # Try to parse as JSON
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        # Try to find JSON object in the text
        brace_start = text.find("{")
        brace_end = text.rfind("}") + 1
        if brace_start != -1 and brace_end > brace_start:
            try:
                return json.loads(text[brace_start:brace_end])
            except json.JSONDecodeError:
                pass

    return None


def call_anthropic(user_command: str) -> Optional[dict]:
    """Call Anthropic Claude API."""
    try:
        import anthropic
        client = anthropic.Anthropic(api_key=LLM_API_KEY)

        message = client.messages.create(
            model=LLM_MODEL,
            max_tokens=500,
            system=SYSTEM_PROMPT,
            messages=[
                {"role": "user", "content": user_command}
            ]
        )

        response_text = message.content[0].text
        return extract_json_from_response(response_text)

    except Exception as e:
        print(f"Anthropic API error: {e}")
        return None


def call_openai(user_command: str) -> Optional[dict]:
    """Call OpenAI GPT API."""
    try:
        import openai
        client = openai.OpenAI(api_key=LLM_API_KEY)

        response = client.chat.completions.create(
            model=LLM_MODEL,
            max_tokens=500,
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": user_command}
            ]
        )

        response_text = response.choices[0].message.content
        return extract_json_from_response(response_text)

    except Exception as e:
        print(f"OpenAI API error: {e}")
        return None


def map_command_to_intent(user_command: str) -> Optional[LLMOutput]:
    """
    Send user command to LLM and parse structured output.
    Returns LLMOutput if successful, None if LLM call or parsing fails.
    """
    # Call appropriate LLM provider
    if LLM_PROVIDER == "anthropic":
        raw_output = call_anthropic(user_command)
    elif LLM_PROVIDER == "openai":
        raw_output = call_openai(user_command)
    else:
        print(f"Unknown LLM provider: {LLM_PROVIDER}")
        return None

    if raw_output is None:
        return None

    # Parse into LLMOutput schema
    try:
        # Convert intent string to enum
        intent_str = raw_output.get("intent", "invalid")
        try:
            intent = Intent(intent_str)
        except ValueError:
            intent = Intent.INVALID
            raw_output["is_valid"] = False
            raw_output["reason"] = f"Unknown intent from LLM: {intent_str}"

        raw_output["intent"] = intent

        return LLMOutput(**raw_output)

    except Exception as e:
        print(f"Failed to parse LLM output: {e}")
        print(f"Raw output was: {raw_output}")
        return None


def get_fallback_output(reason: str) -> LLMOutput:
    """Return a fallback invalid output when LLM fails."""
    return LLMOutput(
        schema_version="1.0",
        is_valid=False,
        intent=Intent.INVALID,
        parameters={},
        confidence=0.0,
        reason=reason
    )
