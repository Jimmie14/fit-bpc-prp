import sys
import re
import time
import select
from collections import deque
from rich.live import Live
from rich.table import Table
from rich.panel import Panel
from rich.console import Console, Group
from rich.text import Text
from rich.align import Align

# Parameters
EXPECTED_SET_FORCE_PER_PUBLISH = 10
ZERO_THRESHOLD = 0.05
MAX_HISTORY = 30  # Number of frequency points to show in the graph
MAX_LOGS = 10     # Number of recent errors to show

SETFORCE_WINDOW_SEC = 2.0
MOTORPUBLISH_WINDOW_SEC = 2.0
SETFORCE_MIN_HZ = 90
MOTORPUBLISH_MIN_HZ = 9

# Regex patterns
setforce_re = re.compile(r"\[manhattan-2\] SetForce: ([\d\.\-e]+), ([\d\.\-e]+)")
publish_re = re.compile(r"\[manhattan-2\] Motor Publish")

# Data structures
setforce_buffer = []
setforce_counts = deque(maxlen=MAX_HISTORY)
log_lines = deque(maxlen=MAX_LOGS)

setforce_times = deque()
motorpublish_times = deque()
setforce_freq_history = deque(maxlen=MAX_HISTORY)
motorpublish_freq_history = deque(maxlen=MAX_HISTORY)
setforce_freq_error_logged = False
motorpublish_freq_error_logged = False

console = Console()

def calc_frequency(times, window_sec):
    now = time.time()
    # Remove old timestamps
    while times and now - times[0] > window_sec:
        times.popleft()
    if len(times) < 2:
        return 0.0
    return (len(times) - 1) / (times[-1] - times[0]) if (times[-1] - times[0]) > 0 else 0.0

def process_line(line):
    global setforce_freq_error_logged, motorpublish_freq_error_logged
    now = time.time()
    if not line.startswith("[manhattan-2]"):
        return
    if setforce_re.search(line):
        m = setforce_re.search(line)
        left = float(m.group(1))
        right = float(m.group(2))
        setforce_buffer.append((left, right))
        setforce_times.append(now)
        # Frequency check
        setforce_hz = calc_frequency(setforce_times, SETFORCE_WINDOW_SEC)
        setforce_freq_history.append(setforce_hz)
        if setforce_hz < SETFORCE_MIN_HZ and not setforce_freq_error_logged:
            log_lines.append((now, f"[red][PROBLEM] SetForce frequency low: {setforce_hz:.2f} Hz (expected ≥ {SETFORCE_MIN_HZ} Hz)[/red]"))
            setforce_freq_error_logged = True
        elif setforce_hz >= SETFORCE_MIN_HZ:
            setforce_freq_error_logged = False
        if abs(left) < ZERO_THRESHOLD and abs(right) < ZERO_THRESHOLD:
            log_lines.append((now, "[red][PROBLEM] SetForce near zero![/red]"))
    elif publish_re.search(line):
        motorpublish_times.append(now)
        n_setforce = len(setforce_buffer)
        setforce_counts.append(n_setforce)
        setforce_buffer.clear()
        # Frequency check
        motorpublish_hz = calc_frequency(motorpublish_times, MOTORPUBLISH_WINDOW_SEC)
        motorpublish_freq_history.append(motorpublish_hz)
        if motorpublish_hz < MOTORPUBLISH_MIN_HZ and not motorpublish_freq_error_logged:
            log_lines.append((now, f"[red][PROBLEM] Motor Publish frequency low: {motorpublish_hz:.2f} Hz (expected ≥ {MOTORPUBLISH_MIN_HZ} Hz)[/red]"))
            motorpublish_freq_error_logged = True
        elif motorpublish_hz >= MOTORPUBLISH_MIN_HZ:
            motorpublish_freq_error_logged = False

def make_log_panel():
    table = Table(show_header=True, header_style="bold magenta")
    table.add_column("Time", width=8)
    table.add_column("Event", width=80)
    for t, msg in log_lines:
        table.add_row(time.strftime("%H:%M:%S", time.localtime(t)), msg)
    return Panel(table, title="Recent Errors", border_style="cyan")

def make_graph_panel():
    # Frequency display
    setforce_hz = setforce_freq_history[-1] if setforce_freq_history else 0.0
    motorpublish_hz = motorpublish_freq_history[-1] if motorpublish_freq_history else 0.0
    setforce_color = "green" if setforce_hz >= SETFORCE_MIN_HZ else "red"
    motorpublish_color = "green" if motorpublish_hz >= MOTORPUBLISH_MIN_HZ else "red"
    # Sparkline for frequency history
    def sparkline(data, color):
        blocks = "▁▂▃▄▅▆▇█"
        if not data:
            return ""
        mn, mx = min(data), max(data)
        if mx == mn:
            return blocks[0] * len(data)
        return "".join(f"[{color}]{blocks[int((v-mn)/(mx-mn)*(len(blocks)-1))]}[/{color}]" for v in data)
    setforce_spark = sparkline(setforce_freq_history, setforce_color)
    motorpublish_spark = sparkline(motorpublish_freq_history, motorpublish_color)
    table = Table(show_header=True, header_style="bold green")
    table.add_column("Event", justify="right")
    table.add_column("Hz", justify="right")
    table.add_column("History", justify="left")
    table.add_row("SetForce", f"[{setforce_color}]{setforce_hz:.2f}[/{setforce_color}]", setforce_spark)
    table.add_row("MotorPub", f"[{motorpublish_color}]{motorpublish_hz:.2f}[/{motorpublish_color}]", motorpublish_spark)
    return Panel(
        Group(
            Align.left(Text("Event Frequencies (Hz, green=OK, red=problem):", style="bold")),
            table,
        ),
        title="Live Frequency Monitor",
        border_style="magenta"
    )

def update_display():
    layout = Group(
        make_log_panel(),
        make_graph_panel()
    )
    return layout

def main():
    with Live(update_display(), refresh_per_second=5, console=console, screen=True) as live:
        while True:
            # Use select to check if there's input to read (non-blocking)
            if select.select([sys.stdin], [], [], 0.1)[0]:
                line = sys.stdin.readline()
                if not line:
                    break
                process_line(line)
                live.update(update_display())
            else:
                # No new input, just refresh the display
                live.update(update_display())

if __name__ == "__main__":
    main()
