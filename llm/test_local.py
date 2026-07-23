"""
Local test script - run without LLM API to verify the system works.
This mocks the LLM response so you can test the full pipeline.
"""
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from backend.schema import LLMOutput, Intent
from backend.validator import validate_llm_output
from backend.queue_store import insert_queue_item, get_all_queue_items, init_db
from backend.schema import QueueItem, QueueStatus


def test_validator():
    """Test the validator with various inputs."""
    print("=" * 50)
    print("Testing Validator")
    print("=" * 50)

    test_cases = [
        # Valid cases
        LLMOutput(
            schema_version="1.0",
            is_valid=True,
            intent=Intent.STATUS,
            parameters={},
            confidence=0.95,
            reason="User asked for status"
        ),
        LLMOutput(
            schema_version="1.0",
            is_valid=True,
            intent=Intent.MOVE_UP,
            parameters={"distance_m": 0.05},
            confidence=0.90,
            reason="User wants to move up"
        ),
        LLMOutput(
            schema_version="1.0",
            is_valid=True,
            intent=Intent.MOVE_UP,
            parameters={"distance_m": 0.08},
            confidence=0.88,
            reason="User specified 8cm"
        ),
        # Invalid: distance too large
        LLMOutput(
            schema_version="1.0",
            is_valid=True,
            intent=Intent.MOVE_UP,
            parameters={"distance_m": 0.50},
            confidence=0.85,
            reason="User wants large movement"
        ),
        # Invalid command
        LLMOutput(
            schema_version="1.0",
            is_valid=False,
            intent=Intent.INVALID,
            parameters={},
            confidence=0.0,
            reason="Not a robot command"
        ),
    ]

    for i, llm_output in enumerate(test_cases):
        result = validate_llm_output(llm_output)
        print(f"\nTest {i + 1}: intent={llm_output.intent.value}")
        print(f"  Validation passed: {result.passed}")
        print(f"  Service command: {result.service_command}")
        if result.error:
            print(f"  Error: {result.error}")
        if result.warnings:
            print(f"  Warnings: {result.warnings}")


def test_queue_store():
    """Test the queue storage."""
    print("\n" + "=" * 50)
    print("Testing Queue Store")
    print("=" * 50)

    # Initialize DB
    init_db()

    # Create a test item
    llm_output = LLMOutput(
        schema_version="1.0",
        is_valid=True,
        intent=Intent.READY,
        parameters={},
        confidence=0.92,
        reason="User wants ready pose"
    )

    validation = validate_llm_output(llm_output)

    item = QueueItem(
        source="text",
        raw_input="go to ready position",
        transcript="go to ready position",
        llm_output=llm_output,
        validation=validation,
        queue_status=QueueStatus.PENDING
    )

    # Insert
    item_id = insert_queue_item(item)
    print(f"\nInserted item with ID: {item_id}")

    # Retrieve all
    items = get_all_queue_items()
    print(f"Total items in queue: {len(items)}")

    for item in items[:3]:  # Show first 3
        print(f"  ID {item['id']}: {item['transcript']} -> {item['llm_output']['intent']}")


def main():
    print("Robot Command Demo - Local Test")
    print("This test does NOT call the LLM API\n")

    test_validator()
    test_queue_store()

    print("\n" + "=" * 50)
    print("Tests complete!")
    print("=" * 50)
    print("\nTo run the full server:")
    print("  1. Copy .env.example to .env")
    print("  2. Add your LLM API key to .env")
    print("  3. Run: uvicorn backend.main:app --reload")
    print("  4. Open: http://127.0.0.1:8000/app")


if __name__ == "__main__":
    main()
