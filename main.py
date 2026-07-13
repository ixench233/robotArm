import sensor, image, time
from pyb import Pin, LED ,Timer,Servo
from pid import PID
from pyb import UART
uart = UART(3,9600)
uart1 = UART(1,9600)
import math
import random
def openlight():
	tim = Timer(4, freq=1000)
	led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)
	led_dac.pulse_width_percent(100)
	led_status=0
red_led   = LED(1)
clock = time.clock()
x=165
lu_x=400-x
lu_y=300-x
rd_x=2*x
rd_y=2*x
roi1	= [lu_x,lu_y,rd_x,rd_y]
TASK=2
def get_gray(x,y,img):
	step_gray=4
	gp1=img.get_pixel(x-step_gray, y-step_gray)
	gp2=img.get_pixel(x-step_gray, y)
	gp3=img.get_pixel(x-step_gray, y+step_gray)
	gp4=img.get_pixel(x, y-step_gray)
	gp5=img.get_pixel(x, y)
	gp6=img.get_pixel(x, y+step_gray)
	gp7=img.get_pixel(x+step_gray, y-step_gray)
	gp8=img.get_pixel(x+step_gray, y)
	gp9=img.get_pixel(x+step_gray, y+step_gray)
	gray=(gp1+gp2+gp3+gp4+gp5+gp6+gp7+gp8+gp9)/9
	return gray
def draw_chess(x,y,img):
	global low_gate
	global up_gate
	if get_gray(x,y,img)<low_gate:
		img.draw_circle(x,y,10,thickness=2,fill=True)
		flag=2
	elif get_gray(x,y,img)>=low_gate and get_gray(x,y,img)<up_gate:
		flag=0
	else:
		img.draw_circle(x,y,10,thickness=2)
		flag=1
	return flag
def find_box_corners(edges):
	if len(edges) < 1:
		return None
	top_left = edges[0]
	top_right = edges[0]
	bottom_left = edges[0]
	bottom_right = edges[0]
	for edge in edges:
		x, y = edge
		if x + y < top_left[0] + top_left[1]:
			top_left = edge
		if x - y > top_right[0] - top_right[1]:
			top_right = edge
		if x - y < bottom_left[0] - bottom_left[1]:
			bottom_left = edge
		if x + y > bottom_right[0] + bottom_right[1]:
			bottom_right = edge
	return top_left, top_right, bottom_left, bottom_right
def get_chessboard_state():
	thresholds1 = ((100, 0))
	sensor.reset()
	sensor.set_pixformat(sensor.GRAYSCALE)
	sensor.set_framesize(sensor.SVGA)
	sensor.skip_frames(time = 33 )
	sensor.set_auto_gain(True)
	sensor.set_auto_whitebal(True)
	img = sensor.snapshot()
	img.lens_corr(strength=1.8, zoom=1.0)
	blobs = img.find_blobs([thresholds1], roi=roi1, pixels_threshold=100, area_threshold=40000,merge=True)
	if blobs:
		for blob in blobs:
			if blob.w()*blob.h()>40000:
				img.draw_edges(blob.min_corners(), thickness=3)
				edges = blob.min_corners()
				top_left, top_right, bottom_left, bottom_right = find_box_corners(edges)
				if top_left is not None:
					img.draw_cross(top_left[0], top_left[1])
					img.draw_cross(top_right[0], top_right[1])
					img.draw_cross(bottom_left[0], bottom_left[1])
					img.draw_cross(bottom_right[0], bottom_right[1])
				point_x=[top_left[0],top_right[0],bottom_left[0],bottom_right[0]]
				point_y=[top_left[1],top_right[1],bottom_left[1],bottom_right[1]]
				length1=math.sqrt((top_left[0]-top_right[0])**2+(top_left[1]-top_right[1])**2)
				length2=math.sqrt((top_right[0]-bottom_right[0])**2+(top_right[1]-bottom_right[1])**2)
				length3=math.sqrt((bottom_left[0]-bottom_right[0])**2+(bottom_left[1]-bottom_right[1])**2)
				length4=math.sqrt((top_left[0]-bottom_left[0])**2+(top_left[1]-bottom_left[1])**2)
				length=(length1+length2+length3+length4)/4
				theta=math.atan2((top_left[1]-top_right[1]),(top_left[0]-top_right[0]))
				x_step=math.cos(theta)*length/3
				y_step=math.sin(theta)*length/3
				p5_x=int((max(point_x)+min(point_x))/2)
				p5_y=int((max(point_y)+min(point_y))/2)
				p4_x=int(p5_x+x_step)
				p4_y=int(p5_y+y_step)
				p6_x=int(p5_x-x_step)
				p6_y=int(p5_y-y_step)
				p2_x=int(p5_x-y_step)
				p2_y=int(p5_y+x_step)
				p8_x=int(p5_x+y_step)
				p8_y=int(p5_y-x_step)
				p1_x=int(p2_x+x_step)
				p1_y=int(p2_y+y_step)
				p3_x=int(p2_x-x_step)
				p3_y=int(p2_y-y_step)
				p7_x=int(p8_x+x_step)
				p7_y=int(p8_y+y_step)
				p9_x=int(p8_x-x_step)
				p9_y=int(p8_y-y_step)
				flg1=draw_chess(p1_x,p1_y,img)
				flg2=draw_chess(p2_x,p2_y,img)
				flg3=draw_chess(p3_x,p3_y,img)
				flg4=draw_chess(p4_x,p4_y,img)
				flg5=draw_chess(p5_x,p5_y,img)
				flg6=draw_chess(p6_x,p6_y,img)
				flg7=draw_chess(p7_x,p7_y,img)
				flg8=draw_chess(p8_x,p8_y,img)
				flg9=draw_chess(p9_x,p9_y,img)
				data=bytearray([0xff,flg1,flg2,flg3,flg4,flg5,flg6,flg7,flg8,flg9,0xfe])
				uart1.write(data)
				print(flg1,flg2,flg3)
				print(flg4,flg5,flg6)
				print(flg7,flg8,flg9)
				xy1 = [p1_x,p1_y]
				xy2 = [p2_x,p2_y]
				xy3 = [p3_x,p3_y]
				xy4 = [p4_x, p4_y]
				xy5 = [p5_x, p5_y]
				xy6 = [p6_x, p6_y]
				xy7 = [p7_x, p7_y]
				xy8 = [p8_x, p8_y]
				xy9 = [p9_x, p9_y]
				location2 = [xy1,xy2,xy3,xy4,xy5,xy6,xy7,xy8,xy9]
				state2 = [flg1,flg2,flg3,flg4,flg5,flg6,flg7,flg8,flg9]
				appppp =  [location2,state2]
		return appppp
def sort1(w_c):
	w_c1 = []
	for i in range(len(w_c)):
		count = 0
		if w_c1:
			for k in range(len(w_c1)):
				if w_c[i][1] <= w_c1[k][1]:
					w_c1.insert(k, w_c[i])
					break
				else:
					count += 1
				if count == len(w_c1):
					w_c1.append(w_c[i])
		else:
			w_c1.append(w_c[i])
	return w_c1
def get_chess_state():
	white_threshold = (83, 100, -96, 127, -96, 105)
	black_threshold = (0, 28, -25, 87, -48, 76)
	roi2 = [544,71,148,462]
	roi3 = [71,73,144,447]
	sensor.reset()
	sensor.set_pixformat(sensor.RGB565)
	sensor.set_framesize(sensor.SVGA)
	sensor.skip_frames(33)
	sensor.set_auto_whitebal(False)
	clock = time.clock()
	wx=[]
	wy=[]
	bx=[]
	by=[]
	clock.tick()
	img = sensor.snapshot()
	img.lens_corr(strength=1.8, zoom=1.0)
	blobs = img.find_blobs([white_threshold],roi=roi2,area_threshold=2000)
	if blobs:
		for b in blobs:
			wx.append(b[5])
			wy.append(b[6])
	blobs = img.find_blobs([black_threshold],roi=roi3,area_threshold=2000)
	if blobs:
		for b in blobs:
			bx.append(b[5])
			by.append(b[6])
	print(wx)
	print(wy)
	print(bx)
	print(bx)
	w_c = []
	b_c = []
	w_cc = []
	b_cc = []
	for i in range(len(wx)):
		w_c.append(wx[i])
		w_c.append(wy[i])
		w_cc.append(w_c)
	for i in range(len(bx)):
		b_c.append(bx[i])
		b_c.append(by[i])
		b_cc.append(b_c)
	b_c1 = sort1(b_cc)
	w_c1 = sort1(w_cc)
	return [w_c1,b_c1]
def send_xy(x1,y1,x2,y2):
	x1_u=x1//255
	x1_l=x1%255
	y1_u=y1//255
	y1_l=y1%255
	x2_u=x2//255
	x2_l=x2%255
	y2_u=y2//255
	y2_l=y2%255
	data=bytearray([x1_u,x1_l,y1_u,y1_l,x2_u,x2_l,y2_u,y2_l])
	uart.write(data)
def judgement(state1):
	count = 0
	for i in state1:
		if i > 0:
			count += 1
	count1 = count%2
	if count1:
		return 2
	else:
		return 1
def chess_state_ai():
	W = []
	B = []
	ALL = []
	[location, state] = get_chessboard_state()
	k = judgement(state)
	[wc,bc] = get_chess_state()
	print("state=",state)
	for i in range(9):
		if state[i] == 0:
			ALL.append(i+1)
		elif state[i] == 1:
			W.append(i+1)
		else:
			B.append(i+1)
	print("ALL,W,B = ",ALL,W,B)
	print("k=",k)
	if k == 1:
		AI = B
		PP = W
		AI_c = bc
	else:
		AI = W
		PP = B
		AI_c = wc
	print("ai=",AI)
	return [k,location,state,AI,PP,ALL,AI_c]
def sum_of_sequence(sequence):
	total = 0
	for element in sequence:
		total += element
	return total
def chess_state_pp():
	W = []
	B = []
	ALL = []
	[location, state] = get_chessboard_state()
	a = judgement(state)
	[wc,bc] = get_chess_state()
	print("state=",state)
	for i in range(9):
		if state[i] == 0:
			ALL.append(i+1)
		elif state[i] == 1:
			W.append(i+1)
		else:
			B.append(i+1)
	print("ALL,W,B = ",ALL,W,B)
	print("a=",a)
	if a == 2:
		AI = B
		PP = W
		PP_c = wc
	else:
		AI = W
		PP = B
		PP_c = bc
	print("pp=",PP)
	return [a,location,state,AI,PP,ALL,PP_c]
def AIplay(Any, weight1):
	max_weight = max(weight1)
	max_index = weight1.index(max_weight)
	return Any[max_index]
def array_complement(big_array, small_array):
	return [element for element in big_array if element not in small_array]
def update_weights(l, ALL, weight, increment):
	directions = [3, 6, -6, -3, -1, 1, 2, -2, 10-l]
	for d in directions:
		if (l + d) in ALL:
			if d == 1 or d == -1:
				if (l - 1) / 3 == int((l - 1) / 3):
					l_index = ALL.index(l + d)
					weight[l_index] -= increment
			if increment == -0.5:
				if ((d == 1 or d == -1) and (l == 2 or l == 8)) or ((d == 3 or d == -3) and (l == 4 or l == 6)):
					l_index = ALL.index(l + d)
					weight[l_index] -= 0.1
			l_index = ALL.index(l + d)
			weight[l_index] += increment
	if l == 5:
		for pos in [1, 3, 7, 9]:
			if pos in ALL:
				l_index = ALL.index(pos)
				weight[l_index] += increment
def weight_d(weight1,past_all,all1,value):
	m = array_complement(past_all, all1)
	if m:
		ppplay = m[0]
		weight1.pop(ppplay-1)
		update_weights(ppplay, aLL1, weight1, value)
	return weight1
def wait():
	while True:
		break
def combinations(iterable, r):
	def _combinations(iterable, r, comb, start):
		if r == 0:
			yield tuple(comb)
		else:
			for i in range(start, len(iterable)):
				comb.append(iterable[i])
				yield from _combinations(iterable, r - 1, comb, i + 1)
				comb.pop()
	return _combinations(iterable, r, [], 0)
def judgment_win(play):
	winning_combinations = [
		[1, 2, 3], [4, 5, 6], [7, 8, 9],
		[1, 4, 7], [2, 5, 8], [3, 6, 9],
		[1, 5, 9], [3, 5, 7]
	]
	if 5 in play:
		pairs = combinations(play, 2)
		if any(sum(pair) == 10 for pair in pairs):
			return 5
	for combo in winning_combinations:
		if all(x in play for x in combo):
			return combo
	return 0
def inspect_moves(moves, ALL, weight, value):
	for k in range(len(ALL)):
		inspection = moves + [ALL[k - 1]]
		if judgment_win(inspection):
			if judgment_win(inspection) == 5:
				pairs = combinations(inspection, 2)
				pairs_with_sum = [pair for pair in pairs if sum(pair) == 10]
				for m in pairs_with_sum:
					intersection = list(set(ALL) & set(m))
					if intersection:
						for d in intersection:
							weight[ALL.index(d)] += value
			else:
				m = judgment_win(inspection)
				intersection = list(set(ALL) & set(m))
				if intersection:
					for d in intersection:
						weight[ALL.index(d)] += value
global low_gate
global up_gate
low_gate=125
up_gate=220
openlight()
problem2_number_num=0
problem2_number_location=[]
problem2_number_chess=[]
problem2_done=0
while 1:
	if uart1.any():
		a = uart1.read()
		print(a[1],a[2],a[3],a[4])
		if a[1]==1:
			[location,state] = get_chessboard_state()
			[w2,b2] = get_chess_state()
			chess_play1 = a[2]
			print(location)
			print(b2[chess_play1-1][0], b2[chess_play1-1][1], location[4][0], location[4][1])
			send_xy(b2[chess_play1-1][0], b2[chess_play1-1][1], location[4][0], location[4][1])
		if a[1]==2 or a[1]==3:
			problem2_number_num=problem2_number_num+1
			problem2_number_chess.append(a[2])
			problem2_number_location.append(a[3])
			print(problem2_number_num)
			print(problem2_number_chess)
			print(problem2_number_location)
			if problem2_number_num==4:
				for i in range(4):
					[location,state] = get_chessboard_state()
					[w2,b2] = get_chess_state()
					if i<2:
						chess_play1 = problem2_number_chess[i]
						location_number =problem2_number_location[i]
						print(b2[chess_play1-1][0], b2[chess_play1-1][1], location[location_number-1][0], location[location_number-1][1])
						send_xy(b2[chess_play1-1][0], b2[chess_play1-1][1], location[location_number-1][0], location[location_number-1][1])
						time.sleep(11)
					if i>1:
						chess_play1 = problem2_number_chess[i]-5
						location_number =problem2_number_location[i]
						print(w2[chess_play1-1][0], w2[chess_play1-1][1], location[location_number-1][0], location[location_number-1][1])
						send_xy(w2[chess_play1-1][0], w2[chess_play1-1][1], location[location_number-1][0], location[location_number-1][1])
						time.sleep(11)
				problem2_done=1
			if problem2_done==1:
				problem2_number_num=0
				problem2_number_location=[]
				problem2_number_chess=[]
				problem2_done=0
		if a[1]==4 or a[1]==5 or a[1]==6:
			weight = weight = [2, 1, 2, 1, 10, 1, 2, 1, 2]
			flag = 0
			past_ALL = [1,2,3,4,5,6,7,8,9]
			past_state = [0,0,0,0,0,0,0,0,0]
			[k,location,state,AI,PP,ALL,AI_c] = chess_state_ai()
			k2 = k
			if k == 1:
				aiplay = a[2]
				print("电脑：",aiplay)
				print( AI_c[0][0], AI_c[0][1],location[aiplay-1][0], location[aiplay-1][1])
				send_xy( AI_c[0][0], AI_c[0][1],location[aiplay-1][0], location[aiplay-1][1])
				time.sleep(11)
				play_index = ALL.index(aiplay)
				past_state = state
				past_ALL = ALL
			while True:
				if flag == 1 or k == 1:
					[a,location,state,AI,PP,ALL,PP_c] = chess_state_pp()
					weight.pop(play_index)
					update_weights(aiplay, ALL, weight, 0.5)
					inspect_moves(AI, ALL, weight, 20000)
					print("该你下了")
					uart1.read()
					while  not uart1.any():
						j=1
					print("下了")
					if len(PP) >= 3 and judgment_win(PP):
						print(PP, "你赢了")
						break
					if not ALL:
						print("双方无棋可下，平局")
						break
					past_state = state
					past_ALL = ALL
					past_k = k
					[k,location,state,AI,PP,ALL,AI_c] = chess_state_ai()
				count = 0
				for i in state:
					count += 1
					if i > past_state[count-1]:
						ppplay = count
						break
				if flag == 0 and k2 == 2 and (ppplay in [1,3,7,9]):
					for i in range(9):
						if (i+1) in [1,3,7,9]:
							weight[i] -=1
						else:
							weight[i] += 1
				print(ppplay,past_ALL)
				play_index = past_ALL.index(ppplay)
				if k == a:
					count2 = 0
					for i in past_state:
						if i == (3-past_k) and state[count2] != i:
							past_ch = count2+1
						if state[count2] == (3-past_k) and state[count2] != i:
							ch = count2+1
						count2 += 1
					print("发现作弊，棋子从",past_ch,"移动到",ch,"处，已纠正")
					print( location[ch-1][0], location[ch-1][1],location[past_ch-1][0], location[past_ch-1][1])
					send_xy( location[ch-1][0], location[ch-1][1],location[past_ch-1][0], location[past_ch-1][1])
					time.sleep(11)
					[a,location,state,AI,PP,ALL,PP_c] = chess_state_pp()
					print("该你下了")
					uart1.read()
					while  not uart1.any():
						j=1
					print("下了")
					if len(PP) >= 3 and judgment_win(PP):
						print(PP, "你赢了")
						break
					if not ALL:
						print("双方无棋可下，平局")
						break
					past_state = state
					past_ALL = ALL
					past_k = k
					[k,location,state,AI,PP,ALL,AI_c] = chess_state_ai()
					count = 0
					for i in state:
						count += 1
						if i > past_state[count-1]:
							ppplay = count
							break
					print(ppplay,past_ALL)
					play_index = past_ALL.index(ppplay)
				weight.pop(play_index)
				update_weights(ppplay, ALL, weight, -0.5)
				inspect_moves(PP, ALL, weight, 100)
				aiplay = AIplay(ALL, weight)
				print("电脑：",aiplay)
				print( AI_c[0][0], AI_c[0][1],location[aiplay-1][0], location[aiplay-1][1])
				send_xy( AI_c[0][0], AI_c[0][1],location[aiplay-1][0], location[aiplay-1][1])
				time.sleep(11)
				play_index = ALL.index(aiplay)
				past_state = state
				print(past_state)
				past_ALL = ALL
				if len(AI) >= 3 and judgment_win(AI):
					print(AI, "电脑获胜")
					break
				if not ALL:
					print("双方无棋可下，平局")
					break
				flag = 1
			uart1.read()
		if a[1]==7:
			global low_gate
			low_gate=a[2]*100+a[3]*10+a[4]
		if a[1]==8:
			global up_gate
			up_gate=a[2]*100+a[3]*10+a[4]
			print(up_gate)