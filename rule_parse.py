from rule_parser.prompts import kg_validation, syntax_validation_prompt, extraction_messages
from rule_parser.query import query_llama, query_gpt
import yaml
import argparse


def knowledge_extraction(rule, model='llama', few_shot=True):
    
    messages = extraction_messages(rule, few_shot)
    
    if model == 'llama':
        response = query_llama(messages)
        return response, messages
    elif model == 'gpt4':
        response = query_gpt(messages)
        return response, messages


def knowledge_validation(history_msgs, initial_ans, model='llama'):
    messages = history_msgs
    messages.append({
        "role": "assistant",
        "content": initial_ans
    })
    messages.append({
        "role": "user",
        "content": kg_validation
    })
    if model == 'llama':
        response = query_llama(messages)
    elif model == 'gpt4':
        response = query_gpt(messages)
    return response, messages

def syntax_validation(response, messages, model='llama'):
    messages.append({
        "role": "assistant",
        "content": response
    })
    messages.append({
        "role": "user",
        "content": syntax_validation_prompt()
    })
    if model == 'llama':
        response = query_llama(messages)
    elif model == 'gpt4':
        response = query_gpt(messages)
    return response, messages

def get_final_output(response, messages, model='llama'):
    messages.append({
        "role": "assistant",
        "content": response
    })

    messages.append({
        "role": "user",
        "content": "Based on the previous responses, output the final representation in YAML format. Do not include any other contents."
    })
    if model == 'llama':
        response = query_llama(messages)
    elif model == 'gpt4':
        response = query_gpt(messages)
    return response
 


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, default='gpt4', help='Model to use: llama or gpt4')
    args = parser.parse_args()
    
    rules_file = open('rule_parser/rules.txt', 'r')
    rules = rules_file.readlines()
    model = args.model

    for i in range(len(rules)):
        rule = rules[i]
        answer, messages = knowledge_extraction(rule, model=model, few_shot=True)
        # print('ke: ', answer)
        answer, messages = knowledge_validation(messages, answer, model=model)
        print('kv: ', answer)
        # dict_ans = extract_values(answer)
        # print(dict_ans)
        ans, messages = syntax_validation(answer, messages, model=model)
        # ans = extract_values(ans)
        print('sv: ', ans)
        final_output = get_final_output(ans, messages, model=model)
        # yaml_ans = yaml.safe_dump(ans)
        # print(yaml_ans)
        print(final_output)
        # save to yaml file
        with open('scenarios/rule_{}.txt'.format(i+1), 'w') as f:
            f.write(final_output)
        if '```yaml' in final_output:
            yaml_ans = final_output.replace('```yaml\n', '').replace('```', '')
        else:
            yaml_ans = final_output.replace('```\n', '').replace('```', '')
            # Parse as Python object
        yaml_dict = yaml.safe_load(yaml_ans)
            
            # Fix the "on" value by ensuring it's treated as a string
        for actor in yaml_dict['Actors'].values():
            if 'Position' in actor and actor['Position'].get('Position relation') is True:
                actor['Position']['Position relation'] = 'on'
        
        # Write to yaml file with explicit string quotes
        with open('scenarios/rule_{}.yaml'.format(i+1), 'w') as f:
            yaml.dump(yaml_dict, f, default_style='"')  # Use default_style='"' to quote strings
