import requests
import re
import openai

openai.api_key = open("openai_key.txt", "r").read().strip()

def query_llama(messages):
    url = "http://localhost:11434/api/chat"
    data = {"messages": messages,
            "model": "llama3.1:70b",
            }
    headers = {"Content-Type": "application/json"}
    response = requests.post(url, json=data, headers=headers)
    content_values = re.findall(r'"content":"(.*?)"', response.text)
    result_text = ''.join(content_values)
    return result_text

def query_gpt(messages, model="gpt-4o"):
    response = openai.ChatCompletion.create(
    model=model,
    messages=messages,
    temperature=1,
    max_tokens=256,
    top_p=1,
    frequency_penalty=0,
    presence_penalty=0
    )
    
    reply = response['choices'][0]['message']['content']

    return reply