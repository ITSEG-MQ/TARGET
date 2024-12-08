import re

# from llm_cost_estimation import count_tokens


def cal_token_usage(text):
    # Split the string into tokens based on whitespace and punctuation

    return len(text) // 4

    
def extract_values(s):
    ans_temp = {
        'Environment': {
            'Weather': None,
            'Time': None
        },
        'Road network': {
            'Road type': None,
            'Road marker': None,
            'Traffic signs': None
        },
        'Actors': {
            'Ego vehicle': {
                'Type': None,
                'Position': {
                    'Position reference': None,
                    'Position relation': None
                },
                'Behavior': None
            },
            'Other actor 1': {
                'Type': None,
                'Position': {
                    'Position reference': None,
                    'Position relation': None
                },
                'Behavior': None
            },
            'Oracle': {
                'longitudinal': None,
                'lateral': None
            }
        }
    }
    # Replace '\n' with actual newlines
    formatted_string = s.replace('\\n', '\n')
    
    # Define a regex pattern to match key-value pairs
    pattern = re.compile(r'^\s*(\w[\w\s]*):\s*(.*)$', re.MULTILINE)
    
    # Find all matches
    matches = pattern.findall(formatted_string)
    
    # Convert matches to a dictionary
    # Convert matches to a dictionary
    values = {key.strip(): value.strip() for key, value in matches}
    
    # Helper function to update nested dictionary
    def update_dict(d, values):
        for key, value in d.items():
            if isinstance(value, dict):
                update_dict(value, values)
            elif key in values:
                d[key] =  re.sub(r'[^a-zA-Z\s]', '', values[key])
    
    # Update ans_temp with extracted values
    update_dict(ans_temp, values)
    
    return ans_temp


if __name__ == "__main__":
    test_str = "Environment:\n  Weather: sunny\n  Time: daytime\nRoad network:\n  Road type: intersection\n  Road marker: None\n  Traffic signs: None\nActors:\n  Ego vehicle:\n    Type: car\n    Position:\n      Position reference: unpaved road\n      Position relation: behind\n    Behavior: go forward\n  Other actor 1:\n    Type: car\n    Position:\n      Position reference: paved road\n      Position relation: front\n    Behavior: go forward\nOracle:\n  longitudinal: yield\n  lateral: None"
    print(cal_token_usage(test_str))