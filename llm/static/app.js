// Robot Command App JavaScript

const commandInput = document.getElementById('commandInput');
const sendBtn = document.getElementById('sendBtn');
const loading = document.getElementById('loading');
const resultSection = document.getElementById('resultSection');

// Allow Enter key to submit
commandInput.addEventListener('keypress', (e) => {
    if (e.key === 'Enter') {
        sendCommand();
    }
});

function setCommand(text) {
    commandInput.value = text;
    commandInput.focus();
}

async function sendCommand() {
    const command = commandInput.value.trim();

    if (!command) {
        alert('Please enter a command');
        return;
    }

    // Show loading, hide result
    loading.classList.add('show');
    resultSection.classList.remove('show');
    sendBtn.disabled = true;

    try {
        const response = await fetch('/api/text-command', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ command: command })
        });

        if (!response.ok) {
            throw new Error(`HTTP error: ${response.status}`);
        }

        const data = await response.json();

        // Update result section
        showResult(data);

        // Clear input
        commandInput.value = '';

    } catch (error) {
        console.error('Error:', error);
        alert('Failed to send command: ' + error.message);
    } finally {
        loading.classList.remove('show');
        sendBtn.disabled = false;
    }
}

function showResult(data) {
    const item = data.item;

    // Update status badge
    const statusBadge = document.getElementById('statusBadge');
    statusBadge.textContent = item.queue_status;
    statusBadge.className = 'status-badge ' + item.queue_status;

    // Update result fields
    document.getElementById('resultTranscript').textContent = item.transcript;
    document.getElementById('resultIntent').textContent = item.intent;
    document.getElementById('resultService').textContent = item.service_command || '(none)';
    document.getElementById('resultReason').textContent = item.reason;

    // Show result section
    resultSection.classList.add('show');
}
