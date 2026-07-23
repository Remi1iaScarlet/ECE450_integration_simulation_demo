"""
FastAPI main application for Robot Command Queue Demo.
"""
from fastapi import FastAPI, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse, HTMLResponse
from pathlib import Path
import json

from .schema import TextCommandRequest, QueueItem, QueueStatus, LLMOutput
from .llm_mapper import map_command_to_intent, get_fallback_output
from .validator import validate_llm_output
from .queue_store import (
    insert_queue_item,
    get_all_queue_items,
    get_queue_item,
    get_posted_queue_item,
    update_queue_status,
    delete_queue_item,
    clear_all_queue_items,
)


app = FastAPI(
    title="Robot Command Queue Demo",
    description="Language-to-structured-command interface for robotic arm control",
    version="1.0.0"
)

# Static files directory
STATIC_DIR = Path(__file__).parent.parent / "static"


# --- Static Pages ---

@app.get("/")
async def root():
    """Redirect to app page."""
    return FileResponse(STATIC_DIR / "app.html")


@app.get("/app")
async def app_page():
    """Mobile-friendly command input page."""
    return FileResponse(STATIC_DIR / "app.html")


@app.get("/queue")
async def queue_page():
    """Queue management page."""
    return FileResponse(STATIC_DIR / "queue.html")


@app.get("/download")
async def download_posted_page():
    """Open a page that downloads the currently posted queue item JSON."""
    html = """<!DOCTYPE html>
<html lang=\"en\">
<head>
    <meta charset=\"UTF-8\">
    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">
    <title>Download Posted Queue Item</title>
</head>
<body>
    <p id="statusText">Preparing download...</p>
    <script>
        fetch('/api/queue/current-posted')
          .then((response) => response.json())
          .then((data) => {
              const item = data.item;
              const statusText = document.getElementById('statusText');
              if (!item) {
                  statusText.textContent = 'No posted item available.';
                  return;
              }

              const blob = new Blob([JSON.stringify(item, null, 2)], { type: 'application/json' });
              const url = URL.createObjectURL(blob);
              const link = document.createElement('a');
              link.href = url;
              link.download = 'queue-item-' + item.id + '.json';
              document.body.appendChild(link);
              link.click();
              statusText.textContent = 'Downloaded item ' + item.id;
              setTimeout(() => URL.revokeObjectURL(url), 1000);
          })
          .catch(() => {
              document.getElementById('statusText').textContent = 'Failed to load posted item.';
          });
    </script>
</body>
</html>"""
    return HTMLResponse(content=html)


# Serve static JS files
@app.get("/static/{filename}")
async def serve_static(filename: str):
    """Serve static files."""
    file_path = STATIC_DIR / filename
    if file_path.exists():
        return FileResponse(file_path)
    raise HTTPException(status_code=404, detail="File not found")


# --- API Endpoints ---

@app.post("/api/text-command")
async def text_command(request: TextCommandRequest):
    """
    Process a text command through the LLM intent mapper.
    Returns the queue item with validation result.
    """
    user_command = request.command.strip()

    # Step 1: Send to LLM for intent mapping
    llm_output = map_command_to_intent(user_command)

    if llm_output is None:
        llm_output = get_fallback_output("LLM call failed or returned invalid JSON")

    # Step 2: Backend validation (never trust LLM's is_valid)
    validation = validate_llm_output(llm_output)

    # Step 3: Determine queue status
    if not validation.passed:
        queue_status = QueueStatus.REJECTED
    elif llm_output.intent.value == "invalid":
        queue_status = QueueStatus.REJECTED
    else:
        queue_status = QueueStatus.PENDING

    # Step 4: Create and store queue item
    queue_item = QueueItem(
        source="text",
        raw_input=user_command,
        transcript=user_command,
        llm_output=llm_output,
        validation=validation,
        queue_status=queue_status
    )

    item_id = insert_queue_item(queue_item)

    # Step 5: Return the created item
    return {
        "success": True,
        "item": {
            "id": item_id,
            "transcript": user_command,
            "intent": llm_output.intent.value,
            "validation_passed": validation.passed,
            "service_command": validation.service_command,
            "queue_status": queue_status.value,
            "reason": llm_output.reason
        }
    }


@app.get("/api/queue")
async def get_queue():
    """Get all queue items."""
    items = get_all_queue_items()
    return {"items": items}


@app.get("/api/queue/current-posted")
async def get_posted_queue_item_api():
    """Get the current posted queue item."""
    item = get_posted_queue_item()
    return {"item": item}


@app.get("/api/queue/{item_id}")
async def get_queue_item_by_id(item_id: int):
    """Get a specific queue item."""
    item = get_queue_item(item_id)
    if item is None:
        raise HTTPException(status_code=404, detail="Queue item not found")
    return item


@app.post("/api/queue/{item_id}/approve")
async def approve_queue_item(item_id: int):
    """Approve a queue item for execution."""
    item = get_queue_item(item_id)
    if item is None:
        raise HTTPException(status_code=404, detail="Queue item not found")

    if item["queue_status"] != "pending":
        raise HTTPException(
            status_code=400,
            detail=f"Cannot approve item with status '{item['queue_status']}'"
        )

    # Check validation passed
    if item["validation"] and not item["validation"].get("passed"):
        raise HTTPException(
            status_code=400,
            detail="Cannot approve item that failed validation"
        )

    current_posted = get_posted_queue_item()
    target_status = QueueStatus.POSTED if current_posted is None else QueueStatus.APPROVED

    success = update_queue_status(item_id, target_status)
    if not success:
        raise HTTPException(status_code=500, detail="Failed to update status")

    return {
        "success": True,
        "message": f"Item {item_id} {'posted' if target_status == QueueStatus.POSTED else 'approved'}",
        "posted": target_status == QueueStatus.POSTED,
        "download_url": "/download" if target_status == QueueStatus.POSTED else None
    }


@app.post("/api/queue/{item_id}/reject")
async def reject_queue_item(item_id: int):
    """Reject a queue item."""
    item = get_queue_item(item_id)
    if item is None:
        raise HTTPException(status_code=404, detail="Queue item not found")

    success = update_queue_status(item_id, QueueStatus.REJECTED)
    if not success:
        raise HTTPException(status_code=500, detail="Failed to update status")

    return {"success": True, "message": f"Item {item_id} rejected"}


@app.delete("/api/queue/{item_id}")
async def delete_queue_item_endpoint(item_id: int):
    """Delete a queue item."""
    success = delete_queue_item(item_id)
    if not success:
        raise HTTPException(status_code=404, detail="Queue item not found")

    return {"success": True, "message": f"Item {item_id} deleted"}


@app.post("/api/queue/clear")
async def clear_queue():
    """Clear all queue items."""
    count = clear_all_queue_items()
    return {"success": True, "message": f"Cleared {count} items"}


@app.post("/api/queue/{item_id}/executed")
async def mark_executed(item_id: int):
    """Mark a queue item as executed (called by robot worker)."""
    item = get_queue_item(item_id)
    if item is None:
        raise HTTPException(status_code=404, detail="Queue item not found")

    success = update_queue_status(item_id, QueueStatus.EXECUTED)
    if not success:
        raise HTTPException(status_code=500, detail="Failed to update status")

    return {"success": True, "message": f"Item {item_id} marked as executed"}


@app.post("/api/queue/{item_id}/failed")
async def mark_failed(item_id: int):
    """Mark a queue item as failed (called by robot worker)."""
    item = get_queue_item(item_id)
    if item is None:
        raise HTTPException(status_code=404, detail="Queue item not found")

    success = update_queue_status(item_id, QueueStatus.FAILED)
    if not success:
        raise HTTPException(status_code=500, detail="Failed to update status")

    return {"success": True, "message": f"Item {item_id} marked as failed"}


# Health check
@app.get("/api/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "ok"}
