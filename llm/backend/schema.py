"""
Pydantic schemas for structured command validation.
"""
from pydantic import BaseModel, Field, field_validator
from typing import Optional, Literal
from enum import Enum
from datetime import datetime


class Intent(str, Enum):
    STATUS = "status"
    READY = "ready"
    OPEN_GRIPPER = "open_gripper"
    CLOSE_GRIPPER = "close_gripper"
    MOVE_UP = "move_up"
    # Semantic tasks (visual_grasp.bridge.v1): routed to /visual_grasp/task
    # instead of /stage2_arm/move. Field names (source_label/container_label/
    # target_xyz/labels) intentionally match multitask/bridge.py's
    # normalize_command() in the visual_grasp repo.
    PICK = "pick"
    PLACE_AT = "place_at"
    PLACE_INTO = "place_into"
    CLEAR_TABLE = "clear_table"
    INVALID = "invalid"


class QueueStatus(str, Enum):
    PENDING = "pending"
    APPROVED = "approved"
    POSTED = "posted"
    REJECTED = "rejected"
    EXECUTED = "executed"
    FAILED = "failed"


class MoveUpParameters(BaseModel):
    distance_m: Optional[float] = Field(default=0.05, ge=0.01, le=0.10)


class LLMOutput(BaseModel):
    """Schema for LLM structured output."""
    schema_version: str = "1.0"
    is_valid: bool
    intent: Intent
    parameters: dict = Field(default_factory=dict)
    confidence: float = Field(ge=0.0, le=1.0)
    reason: str

    @field_validator("parameters")
    @classmethod
    def validate_parameters(cls, v, info):
        return v if v else {}


class ValidationResult(BaseModel):
    """Result of backend validation."""
    passed: bool
    service_name: str = "/stage2_arm/move"
    service_command: Optional[str] = None
    warnings: list[str] = Field(default_factory=list)
    error: Optional[str] = None


class QueueItem(BaseModel):
    """Complete queue item stored in database."""
    id: Optional[int] = None
    created_at: Optional[str] = None
    source: Literal["text", "voice"] = "text"
    raw_input: str
    transcript: str
    llm_output: Optional[LLMOutput] = None
    validation: Optional[ValidationResult] = None
    queue_status: QueueStatus = QueueStatus.PENDING


class TextCommandRequest(BaseModel):
    """Request body for text command API."""
    command: str = Field(min_length=1, max_length=500)


class QueueItemResponse(BaseModel):
    """Response for queue item."""
    id: int
    created_at: str
    source: str
    raw_input: str
    transcript: str
    llm_output: Optional[dict] = None
    validation: Optional[dict] = None
    queue_status: str
