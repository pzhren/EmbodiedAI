import os
from openai import AzureOpenAI,OpenAI
import requests
import base64
import cv2
import numpy as np
from mimetypes import guess_type

API_KEY = ""

def local_image_to_data_url(image):
    if isinstance(image,str):
        mime_type, _ = guess_type(image)
        with open(image, "rb") as image_file:
            base64_encoded_data = base64.b64encode(image_file.read()).decode('utf-8')
        return f"data:{mime_type};base64,{base64_encoded_data}"
    elif isinstance(image,np.ndarray):
        base64_encoded_data = base64.b64encode(cv2.imencode('.jpg',image)[1]).decode('utf-8')
        return f"data:image/jpeg;base64,{base64_encoded_data}"


# 大模型口接口
import base64
import requests
from tenacity import (
    retry,
    stop_after_attempt,
    wait_random_exponential,
)

@retry(wait=wait_random_exponential(min=5, max=60), stop=stop_after_attempt(10))
def gptv_response(text_prompt,image_prompt,system_prompt=""):
    api_key = API_KEY
    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {api_key}"
    }
    
    prompt = [{'role':'system','content':system_prompt},
             {'role':'user','content':[{'type':'text','text':text_prompt},
                                       {'type':'image_url','image_url':{'url':local_image_to_data_url(image_prompt)}}]}]
    payload = {
        "model": "gpt-4o-mini",
        "messages": prompt
    }
    response = requests.post("https://api.openai.com/v1/chat/completions", headers=headers, json=payload)

    output = response.json()
    # print(output["usage"])
    # print(output)
    # print(output["choices"][0]['message'])
    return output["choices"][0]['message']["content"]

@retry(wait=wait_random_exponential(min=5, max=60), stop=stop_after_attempt(10))
def gpt_response(text_prompt,system_prompt=""):
    api_key = API_KEY
    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {api_key}"
    }
    
    prompt = [{'role':'system','content':system_prompt},
              {'role':'user','content':[{'type':'text','text':text_prompt}]}]
    
    payload = {
        "model": "gpt-4o-mini",
        "messages": prompt
    }
    response = requests.post("https://api.openai.com/v1/chat/completions", headers=headers, json=payload)

    output = response.json()
    # print(output["usage"])
    # print(output)
    # print(output["choices"][0]['message'])
    return output["choices"][0]['message']["content"]
