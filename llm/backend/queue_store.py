"""
SQLite queue storage for robot commands.
"""
import sqlite3
import json
import os
from datetime import datetime
from typing import Optional
from contextlib import contextmanager
from .config import DATABASE_PATH
from .schema import QueueItem, QueueStatus, LLMOutput, ValidationResult


def get_db_path() -> str:
    """Get database path, creating directory if needed."""
    db_dir = os.path.dirname(DATABASE_PATH)
    if db_dir and not os.path.exists(db_dir):
        os.makedirs(db_dir)
    return DATABASE_PATH


@contextmanager
def get_connection():
    """Context manager for database connection."""
    conn = sqlite3.connect(get_db_path())
    conn.row_factory = sqlite3.Row
    try:
        yield conn
    finally:
        conn.close()


def init_db():
    """Initialize database schema."""
    with get_connection() as conn:
        conn.execute("""
            CREATE TABLE IF NOT EXISTS command_queue (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                created_at TEXT NOT NULL,
                source TEXT NOT NULL,
                raw_input TEXT NOT NULL,
                transcript TEXT NOT NULL,
                llm_output TEXT,
                validation TEXT,
                queue_status TEXT NOT NULL DEFAULT 'pending'
            )
        """)
        conn.commit()


def insert_queue_item(item: QueueItem) -> int:
    """Insert a new queue item, returns the new item ID."""
    with get_connection() as conn:
        cursor = conn.execute("""
            INSERT INTO command_queue
            (created_at, source, raw_input, transcript, llm_output, validation, queue_status)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        """, (
            datetime.now().isoformat(),
            item.source,
            item.raw_input,
            item.transcript,
            json.dumps(item.llm_output.model_dump() if item.llm_output else None),
            json.dumps(item.validation.model_dump() if item.validation else None),
            item.queue_status.value
        ))
        conn.commit()
        return cursor.lastrowid


def get_all_queue_items() -> list[dict]:
    """Get all queue items, ordered by created_at descending."""
    with get_connection() as conn:
        cursor = conn.execute("""
            SELECT * FROM command_queue ORDER BY created_at DESC
        """)
        rows = cursor.fetchall()

        items = []
        for row in rows:
            item = dict(row)
            # Parse JSON fields
            if item["llm_output"]:
                item["llm_output"] = json.loads(item["llm_output"])
            if item["validation"]:
                item["validation"] = json.loads(item["validation"])
            items.append(item)

        return items


def get_queue_item(item_id: int) -> Optional[dict]:
    """Get a single queue item by ID."""
    with get_connection() as conn:
        cursor = conn.execute("""
            SELECT * FROM command_queue WHERE id = ?
        """, (item_id,))
        row = cursor.fetchone()

        if row is None:
            return None

        item = dict(row)
        if item["llm_output"]:
            item["llm_output"] = json.loads(item["llm_output"])
        if item["validation"]:
            item["validation"] = json.loads(item["validation"])

        return item


def get_posted_queue_item() -> Optional[dict]:
    """Get the single currently posted queue item, if any."""
    with get_connection() as conn:
        cursor = conn.execute("""
            SELECT * FROM command_queue
            WHERE queue_status = 'posted'
            ORDER BY created_at ASC, id ASC
            LIMIT 1
        """)
        row = cursor.fetchone()

        if row is None:
            return None

        item = dict(row)
        if item["llm_output"]:
            item["llm_output"] = json.loads(item["llm_output"])
        if item["validation"]:
            item["validation"] = json.loads(item["validation"])

        return item


def get_next_approved_queue_item() -> Optional[dict]:
    """Get the oldest approved queue item, used when promoting after delete."""
    with get_connection() as conn:
        cursor = conn.execute("""
            SELECT * FROM command_queue
            WHERE queue_status = 'approved'
            ORDER BY created_at ASC, id ASC
            LIMIT 1
        """)
        row = cursor.fetchone()

        if row is None:
            return None

        item = dict(row)
        if item["llm_output"]:
            item["llm_output"] = json.loads(item["llm_output"])
        if item["validation"]:
            item["validation"] = json.loads(item["validation"])

        return item


def update_queue_status(item_id: int, status: QueueStatus) -> bool:
    """Update the status of a queue item."""
    with get_connection() as conn:
        cursor = conn.execute("""
            UPDATE command_queue SET queue_status = ? WHERE id = ?
        """, (status.value, item_id))
        conn.commit()
        return cursor.rowcount > 0


def promote_next_approved_to_posted() -> Optional[int]:
    """Promote the oldest approved item to posted and return its ID."""
    with get_connection() as conn:
        cursor = conn.execute("""
            SELECT id FROM command_queue
            WHERE queue_status = 'approved'
            ORDER BY created_at ASC, id ASC
            LIMIT 1
        """)
        row = cursor.fetchone()

        if row is None:
            return None

        item_id = row[0]
        conn.execute("""
            UPDATE command_queue SET queue_status = 'posted' WHERE id = ?
        """, (item_id,))
        conn.commit()
        return item_id


def delete_queue_item(item_id: int) -> bool:
    """Delete a queue item."""
    with get_connection() as conn:
        cursor = conn.execute("SELECT queue_status FROM command_queue WHERE id = ?", (item_id,))
        row = cursor.fetchone()
        if row is None:
            return False

        deleted_status = row[0]

        cursor = conn.execute("""
            DELETE FROM command_queue WHERE id = ?
        """, (item_id,))
        conn.commit()

    if deleted_status == 'posted':
        promote_next_approved_to_posted()

    return cursor.rowcount > 0


def clear_all_queue_items() -> int:
    """Delete all queue items, returns count deleted."""
    with get_connection() as conn:
        cursor = conn.execute("DELETE FROM command_queue")
        conn.commit()
        return cursor.rowcount


def get_approved_pending_execution() -> list[dict]:
    """Get all approved items that haven't been executed yet."""
    with get_connection() as conn:
        cursor = conn.execute("""
            SELECT * FROM command_queue
            WHERE queue_status = 'approved'
            ORDER BY created_at ASC
        """)
        rows = cursor.fetchall()

        items = []
        for row in rows:
            item = dict(row)
            if item["llm_output"]:
                item["llm_output"] = json.loads(item["llm_output"])
            if item["validation"]:
                item["validation"] = json.loads(item["validation"])
            items.append(item)

        return items


# Initialize database on module import
init_db()
