"""
Simple utility hooks for common tasks
"""


def create_status_printer(every_n_ticks: int = 10):
    """Print status every N ticks"""

    def print_status(result, blackboard, py_tree):
        if result.tick_count % every_n_ticks == 0:
            print(f"[Tick {result.tick_count}] {result.status}")

    return print_status
