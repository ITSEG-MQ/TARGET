
dsl_definition = '''
    The DSL defines a test scenario using four main fields, 'environment', 'road network', 'actors', and 'oracle'.
    'Environment' contains two subfields 'weather' and 'time'  to present when the test scenario occurs, what the current weather and time are. 
    'Road network' characterizes the geographical context of the test scenario, defining the road type, and the presence of the road marker and traffic signs.
    'Actor' describes dynamic objects involved in the test scenario including the 'ego vehicle' and 'other actors'. For each actor, its 'type', 'position', and current 'behavior' when the scenario occurs are defined. Specifically, 'position' is composed of two subfields 'position reference' and 'position relation' to describe the relative position of an actor regarding the ego vehicle or road network elements.
    'Oracle' regulates the appropriate behaviors of the ego vehicle in 'longitudinal' and 'lateral' directions during the test scenario. 

    Then I'll provide the detailed rules for the DSL
    '''

dsl_rules = '''
    For each subfield, I create a list of optional elements and assignment rules. If the element mentioned in the traffic rule for a subfield is close to one of the elements in the provided optional element list, set the element in the optional element list for the subfield. If the element mentioned in the traffic rule is not defined in the optional element list, keep the element you identified in the traffic rule as the element for the subfield. If  the element for a subfield is not mentioned, there are some rules to define the default element for the subfield. Next I will provide the optional element lists and default element rules for each subfield:
1. For 'weather', the optional element list is ['sunny', 'rainy', 'foggy', 'snowy']. If the weather is not mentioned in the traffic rule, set the element of 'weather' as 'sunny'.
2. For 'time', the optional element is ['daytime', 'nighttime']. If the time is not mentioned in the traffic rule, set the element of the 'time' as 'daytime' .
3. For 'road type', the optional element list contains ['intersection', 'roundabout', 't-intersection', 'highway', 'straight road', 'two-lane road', 'crosswalk', 'railway'] but not limited to these elements. If the road type mentioned in the traffic rule is not in the optional list, still set the element as the road type mentioned in the traffic rule. If the road type is not mentioned in the traffic rule, set the element as 'straight road'.
4. For 'road marker', the optional element list is ['solid line', 'broken line']. If the road marker is not mentioned in the traffic rule, set the element as None for the subfield.
5. For 'traffic signs', the optional element list is ['traffic light', 'speed limit sign', 'yield sign', 'stop sign', 'no turn left sign', 'do not enter sign']. If the traffic signs are not mentioned in the traffic rule, set the element as None for the subfield.
6.  For 'type', the optional element list is ['car', 'truck', 'train']. If the element is not mentioned in the traffic rule, set the element as 'car' for the subfield.
7. For 'position reference', the element could be one of the 'road type'  or 'traffic sign' elements mentioned in the traffic rule or 'ego vehicle'.
  7.1 If the actor is 'ego vehicle', the 'position reference' can only be the 'road type' or 'traffic sign' elements. If the position relationship between the ego vehicle and 'road type' or 'traffic sign' elements are not mentioned in the traffic rule, set 'position reference' as None.
  7.2 if the actor is not 'ego vehicle', the element could be one of the 'road type'  or 'traffic sign' elements mentioned in the traffic rule or 'ego vehicle'.
8. For 'position relation', the common optional element list is ['front', 'behind', 'left', 'right', 'opposite']
  8.1. If the 'position reference' is 'ego vehicle,  'front' and 'behind' means the other actor is ahead of back to the ego vehicle. 'left' and 'right' mean the other actor is on the left or right lane relative to the ego vehicle. 'opposite' means the other actor is on the opposite road relative to the ego vehicle. 
 8.2. If the 'position reference' is an element of the 'road type' or 'traffic signs', 'front' means the other actor has passed the 'road type' or 'traffic sign' element. 'behind' means the actor is approaching the 'road type' or 'traffic sign' element. 'left' and 'right' means the actor is located at the left or right side of the road type or traffic sign from the perspective of the ego vehicle. 'opposite' means the other actor is approaching the road network from the opposite direction. If the 'position reference' is an element of the 'road type', another optional element for 'position relation' is 'on', which means the actor is on the 'road type' element or inside the 'road type' element.
9. For 'behavior', the optional element list is ['go forward', 'turn left', 'turn right', 'decelerate', 'change lane to left', 'change lane to right', 'stopped']. If the element is not mentioned in the traffic rule, set the element as 'go forward' for the subfield.
10. For 'longitudinal oracle', the optional element list is ['yield', 'stop', 'decelerate', 'accelerate']. If not mentioned in the traffic rule, set the element as 'None' for the subfield.
11. For 'lateral oracle', the optional element list is ['turn left', 'turn right', 'keep lane', 'change lane to left', 'change lane to right']. If not mentioned in the traffic rule, set the element as 'None' for the subfield.
'''

example_dsl = '''
Now I will first provide you an example of the traffic rule and the corresponding test scenario following the DSL syntax. Then I will give you the traffic rule to process.

Example traffic rule: "when approaching intersections not controlled by signs, signals, multi-Lanes, or pavement, yield the right-of-way to any vehicle that has entered or is approaching the intersection on your right".

Example test scenario in YAML:
```
Environment:
   Weather: sunny
   Time: daytime
Road network:
  Road type: intersection
  Road marker: None
  Traffic signs: None
Actors:
  Ego vehicle:
    Type: car
    Position:
        Position reference: intersection
       Position relation: behind
    Behavior: go forward
  Other actor 1:
    Type: car
    Position:
        Position reference: intersection
        Position relation: right
    Behavior: go forward
  Oracle:
     longitudinal: yield
    lateral: None
```

Do you understand the given example? I will provide the traffic rule to process now.
'''

kg_validation = "Are the elements in the generated scenario description consistent with the input traffic rule? If not, correct the inconsistencies and output the revised scenario representation."

def syntax_validation_prompt():
    return "For each element in your output driving scenario representation, find out an element with the closest meaning in the provided DSL rules. If all element options are not close to the given element, keep the given element. Finally output the revised driving scenario representation."

def extraction_messages(rule, few_shot=True):
    question = '''
        Traffic rule: {}

        Only output the test scenario in YAML syntax without any other contents 
        '''.format(rule)
    if few_shot:
        messages=[
            {
            "role": "system",
            "content": "You are a test expert for autonomous driving systems. Your task is to derive test scenarios from given traffic rules."
            },
            {
            "role": "user",
            "content": dsl_definition
            },
            {
            "role": "assistant",
            "content": "Thank you for providing the DSL definition. Please go ahead and provide the detailed traffic rules for each of the fields ('environment,' 'road network,' 'actors,' and 'oracle'), and I will assist in deriving test scenarios based on those rules for autonomous driving systems."
            },
            {
            "role": "user",
            "content": dsl_rules
            },
            {
            "role": "assistant",
            "content": "Thank you for providing the optional element lists and rules for each subfield of the DSL. With these guidelines in mind, I can now help you create test scenarios based on traffic rules. Please provide the traffic rules you'd like to use, and I will use the provided rules to generate test scenarios for autonomous driving systems."
            },
            {
            "role": "user",
            "content": example_dsl
            },
            {
            "role": "assistant",
            "content": "Yes, I understand the example provided, and I can see how it corresponds to the DSL syntax. Please go ahead and provide the traffic rule you would like me to use to create a test scenario following the DSL syntax, and I will generate the test scenario accordingly."
            },
            {"role": "user",
            "content": question
            }
        ]
    else:
        messages=[
            {
            "role": "system",
            "content": "You are a test expert for autonomous driving systems. Your task is to derive test scenarios from given traffic rules."
            },
            {
            "role": "user",
            "content": dsl_definition
            },
            {
            "role": "assistant",
            "content": "Thank you for providing the DSL definition. Please go ahead and provide the detailed traffic rules for each of the fields ('environment,' 'road network,' 'actors,' and 'oracle'), and I will assist in deriving test scenarios based on those rules for autonomous driving systems."
            },
            {
            "role": "user",
            "content": dsl_rules
            },
            {
            "role": "assistant",
            "content": "Thank you for providing the optional element lists and rules for each subfield of the DSL. With these guidelines in mind, I can now help you create test scenarios based on traffic rules. Please provide the traffic rules you'd like to use, and I will use the provided rules to generate test scenarios for autonomous driving systems."
            },
            {
            "role": "user",
            "content": question
            }
        ]
    return messages    