from inputs import get_gamepad

print("=== Move each stick / press each button and watch the output ===")

while True:
    events = get_gamepad()
    for e in events:
        print(f"code={e.code:12s}, state={e.state}")
