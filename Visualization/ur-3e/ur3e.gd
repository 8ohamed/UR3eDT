extends Node3D

@onready var rmq_connection = get_node("rmq_listener")

func _ready() -> void:
	rmq_connection.connect("OnMessage", _on_message)

func _on_message(message):
	var data = JSON.parse_string(message)
	#print(data)
	
	var q_actual = data.q_actual
	if q_actual:
		print(q_actual)
		var joint_0_val = %UR3e/b/mk_0 as Node3D
		joint_0_val.rotation.z =  - q_actual[0]
		#joint_0_val.transform3D()
		
		var joint_1_val = %UR3e/b/mk_0/j0/mk_1 as Marker3D
		joint_1_val.rotation.y = - q_actual[1] - PI / 2
		
		var joint_2_val = %UR3e/b/mk_0/j0/mk_1/j1/mk_2 as Marker3D
		joint_2_val.rotation.y = -q_actual[2]

		var joint_3_val = %UR3e/b/mk_0/j0/mk_1/j1/mk_2/j2/mk_3 as Marker3D
		joint_3_val.rotation.y = - q_actual[3] - PI / 2

		var joint_4_val = %UR3e/b/mk_0/j0/mk_1/j1/mk_2/j2/mk_3/j3/mk_4 as Marker3D
		joint_4_val.rotation.z = - q_actual[4]

		var joint_5_val = %UR3e/b/mk_0/j0/mk_1/j1/mk_2/j2/mk_3/j3/mk_4/j4/mk_5 as Marker3D
		joint_5_val.rotation.y = - q_actual[5]
