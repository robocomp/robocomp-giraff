import math


hits_to_reach_top_thr = 10 # number or hits before getting 0.7
min_lambda_value = -30
max_lambda_value = 30
top_thr = 0.7 # threshold value for next level
bot_thr = 0.2 # threshold value for REMOVING
s = -hits_to_reach_top_thr / (math.log(1.0 / top_thr - 1.0))
integrator = lambda x: 1.0 / (1.0 + math.exp((-int(x) / s))) # 1.0 / (1.0 + exp(-x / s))


for i in range(-20, 21):
    print("integrator(" + str(i) + ") ->  " + str(integrator(i)))


