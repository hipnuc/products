import json
import numpy as np
from collections import defaultdict

def analyze_frame_drops(filename):
    """
    Analyze frame drops for each node_id based on timestamp differences
    """
    # Read JSON data
    with open(filename, 'r') as f:
        data = [json.loads(line) for line in f]
    
    # Group data by node_id
    node_data = defaultdict(list)
    for entry in data:
        node_id = entry['node_id']
        timestamp = entry['hw_ts_us']
        node_data[node_id].append(timestamp)
    
    # Sort timestamps for each node
    for node_id in node_data:
        node_data[node_id].sort()
    
    # Analyze time differences
    results = {}
    for node_id, timestamps in sorted(node_data.items()):
        if len(timestamps) < 2:
            continue
            
        # Calculate time differences (in microseconds)
        time_diffs = np.diff(timestamps)
        
        # Convert to milliseconds for readability
        time_diffs_ms = time_diffs / 1000.0
        
        # Statistics
        results[node_id] = {
            'total_frames': len(timestamps),
            'mean_interval_ms': np.mean(time_diffs_ms),
            'std_interval_ms': np.std(time_diffs_ms),
            'min_interval_ms': np.min(time_diffs_ms),
            'max_interval_ms': np.max(time_diffs_ms),
            'expected_interval_ms': np.median(time_diffs_ms),  # Use median as expected
        }
        
        # Detect frame drops (intervals > 1.5x expected)
        expected = results[node_id]['expected_interval_ms']
        threshold = expected * 1.5
        drops = time_diffs_ms[time_diffs_ms > threshold]
        
        results[node_id]['drop_count'] = len(drops)
        results[node_id]['drop_intervals_ms'] = drops.tolist() if len(drops) > 0 else []
        results[node_id]['drop_rate'] = len(drops) / len(time_diffs) * 100
    
    return results

def print_analysis(results):
    """
    Pretty print the analysis results
    """
    print("=" * 80)
    print("IMU Frame Drop Analysis")
    print("=" * 80)
    
    for node_id, stats in sorted(results.items()):
        print(f"\n📊 Node ID: {node_id}")
        print(f"   Total Frames: {stats['total_frames']}")
        print(f"   Expected Interval: {stats['expected_interval_ms']:.2f} ms")
        print(f"   Mean Interval: {stats['mean_interval_ms']:.2f} ± {stats['std_interval_ms']:.2f} ms")
        print(f"   Min/Max Interval: {stats['min_interval_ms']:.2f} / {stats['max_interval_ms']:.2f} ms")
        print(f"   Frame Drops: {stats['drop_count']} ({stats['drop_rate']:.2f}%)")
        
        if stats['drop_intervals_ms']:
            print(f"   Drop Intervals (ms): {[f'{x:.2f}' for x in stats['drop_intervals_ms'][:5]]}")
            if len(stats['drop_intervals_ms']) > 5:
                print(f"   ... and {len(stats['drop_intervals_ms']) - 5} more")
    
    print("\n" + "=" * 80)

if __name__ == "__main__":
    # Analyze the data
    results = analyze_frame_drops('imu.json')
    
    # Print summary
    print_analysis(results)
    
