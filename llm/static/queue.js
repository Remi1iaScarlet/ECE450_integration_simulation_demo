// Command Queue Management JavaScript

let queueData = [];

// Load queue on page load
document.addEventListener('DOMContentLoaded', loadQueue);

// Auto-refresh every 5 seconds
setInterval(loadQueue, 5000);

async function loadQueue() {
    try {
        const response = await fetch('/api/queue');
        if (!response.ok) throw new Error('Failed to load queue');

        const data = await response.json();
        queueData = data.items;

        renderQueue();
        updateStats();

    } catch (error) {
        console.error('Error loading queue:', error);
    }
}

function updateStats() {
    const total = queueData.length;
    const pending = queueData.filter(i => i.queue_status === 'pending').length;
    const approved = queueData.filter(i => i.queue_status === 'approved').length;
    const posted = queueData.filter(i => i.queue_status === 'posted').length;
    const executed = queueData.filter(i => i.queue_status === 'executed').length;

    document.getElementById('statTotal').textContent = total;
    document.getElementById('statPending').textContent = pending;
    document.getElementById('statApproved').textContent = approved;
    const statPosted = document.getElementById('statPosted');
    if (statPosted) {
        statPosted.textContent = posted;
    }
    document.getElementById('statExecuted').textContent = executed;
}

function renderQueue() {
    const container = document.getElementById('queueContainer');

    if (queueData.length === 0) {
        container.innerHTML = `
            <div class="empty-state">
                <h3>No commands in queue</h3>
                <p>Commands sent from the app will appear here</p>
            </div>
        `;
        return;
    }

    const rows = queueData.map(item => {
        const time = new Date(item.created_at).toLocaleString();
        const intent = item.llm_output?.intent || 'unknown';
        const serviceCmd = item.validation?.service_command || '-';

        return `
            <tr>
                <td class="time-cell">${time}</td>
                <td class="transcript-cell" title="${escapeHtml(item.transcript)}">${escapeHtml(item.transcript)}</td>
                <td><span class="intent-badge">${intent}</span></td>
                <td>${serviceCmd}</td>
                <td><span class="status-badge ${item.queue_status}">${item.queue_status}</span></td>
                <td class="actions-cell">
                    ${getActionButtons(item)}
                if (data.posted && data.download_url) {
                    window.open(data.download_url, '_blank', 'noopener');
            </tr>
        `;
    }).join('');

    container.innerHTML = `
        <table class="queue-table">
            <thead>
                <tr>
                    <th>Time</th>
                    <th>Transcript</th>
                    <th>Intent</th>
                    <th>Service Cmd</th>
                    <th>Status</th>
                    <th>Actions</th>
                </tr>
            </thead>
            <tbody>
                ${rows}
            </tbody>
        </table>
    `;
}

function getActionButtons(item) {
    let buttons = `<button class="btn btn-secondary btn-small" onclick="showDetail(${item.id})">View</button>`;

    if (item.queue_status === 'pending') {
        buttons += `<button class="btn btn-primary btn-small" onclick="approveItem(${item.id})">Approve</button>`;
        buttons += `<button class="btn btn-danger btn-small" onclick="rejectItem(${item.id})">Reject</button>`;
    }

    buttons += `<button class="btn btn-danger btn-small" onclick="deleteItem(${item.id})">Delete</button>`;
    if (item.queue_status === 'posted') {
        buttons += `<a class="btn btn-secondary btn-small" href="/download" target="_blank" rel="noopener">Download JSON</a>`;
    }

    return buttons;
}

async function approveItem(id) {
    try {
        const response = await fetch(`/api/queue/${id}/approve`, { method: 'POST' });
        if (!response.ok) {
            const err = await response.json();
            throw new Error(err.detail || 'Failed to approve');
        }
        const data = await response.json();
        loadQueue();
        if (data.download_url) {
            window.open(data.download_url, '_blank', 'noopener');
        }
    } catch (error) {
        alert('Error: ' + error.message);
    }
}

async function rejectItem(id) {
    try {
        const response = await fetch(`/api/queue/${id}/reject`, { method: 'POST' });
        if (!response.ok) throw new Error('Failed to reject');
        loadQueue();
    } catch (error) {
        alert('Error: ' + error.message);
    }
}

async function deleteItem(id) {
    if (!confirm('Delete this command?')) return;

    try {
        const response = await fetch(`/api/queue/${id}`, { method: 'DELETE' });
        if (!response.ok) throw new Error('Failed to delete');
        loadQueue();
    } catch (error) {
        alert('Error: ' + error.message);
    }
}

async function clearQueue() {
    if (!confirm('Clear all commands? This cannot be undone.')) return;

    try {
        const response = await fetch('/api/queue/clear', { method: 'POST' });
        if (!response.ok) throw new Error('Failed to clear');
        loadQueue();
    } catch (error) {
        alert('Error: ' + error.message);
    }
}

function showDetail(id) {
    const item = queueData.find(i => i.id === id);
    if (!item) return;

    const modal = document.getElementById('detailModal');
    const body = document.getElementById('modalBody');

    body.innerHTML = `
        <div class="detail-section">
            <div class="detail-label">ID</div>
            <div class="detail-value">${item.id}</div>
        </div>
        <div class="detail-section">
            <div class="detail-label">Created At</div>
            <div class="detail-value">${item.created_at}</div>
        </div>
        <div class="detail-section">
            <div class="detail-label">Source</div>
            <div class="detail-value">${item.source}</div>
        </div>
        <div class="detail-section">
            <div class="detail-label">Transcript</div>
            <div class="detail-value">${escapeHtml(item.transcript)}</div>
        </div>
        <div class="detail-section">
            <div class="detail-label">LLM Output</div>
            <div class="detail-value">${JSON.stringify(item.llm_output, null, 2)}</div>
        </div>
        <div class="detail-section">
            <div class="detail-label">Validation</div>
            <div class="detail-value">${JSON.stringify(item.validation, null, 2)}</div>
        </div>
        <div class="detail-section">
            <div class="detail-label">Queue Status</div>
            <div class="detail-value">${item.queue_status}</div>
        </div>
    `;

    modal.classList.add('show');
}

function closeModal() {
    document.getElementById('detailModal').classList.remove('show');
}

// Close modal on escape key
document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') closeModal();
});

// Close modal on outside click
document.getElementById('detailModal').addEventListener('click', (e) => {
    if (e.target.id === 'detailModal') closeModal();
});

function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
}
