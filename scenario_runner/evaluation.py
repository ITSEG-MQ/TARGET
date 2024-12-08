import pandas as pd
import numpy as np
from collections import Counter

def rq3():
    models = ['auto', 'mmfn', 'lav', 'carla_ad']

    for model in models:
        result_file = open('result_{}.txt'.format(model), 'r')
        result_raw = result_file.readlines()
        result_dict = {}
        for line in result_raw:
            result = line.split(':')
            yaml = result[0]
            exp_map = result[1]
            violations = result[2]
            # print(yaml, violations)
            if yaml not in result_dict:
                result_dict[yaml] = {'exp_count': 1}
            else:
                if exp_map not in result_dict[yaml]:
                    result_dict[yaml][exp_map] = eval(violations)
                    result_dict[yaml]['exp_count'] += 1
                else:
                    result_dict[yaml][exp_map] += (eval(violations))
                    result_dict[yaml]['exp_count'] += 1

        print(result_dict)
        for rule,result in result_dict.items():
            print(rule)
            violations_total = []
            for k, v in result.items():
                if k != 'exp_count':
                    # print(k)
                    violations_total += v
                    # violations_counter = Counter(v)
                    # print(violations_counter)
            violations_counter = Counter(violations_total)
            print(violations_counter)
        break

rq3()