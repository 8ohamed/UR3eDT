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
		var j0 = %UR3e/b/mk_0 as Marker3D
		j0.rotation.z =  - q_actual[0]
		
		var j1 = %UR3e/b/mk_0/j0/mk_1 as Marker3D
		j1.rotation.y = - q_actual[1] - PI / 2
		
		var j2 = %UR3e/b/mk_0/j0/mk_1/j1/mk_2 as Marker3D
		j2.rotation.y = -q_actual[2]

		var j3 = %UR3e/b/mk_0/j0/mk_1/j1/mk_2/j2/mk_3 as Marker3D
		j3.rotation.y = - q_actual[3] - PI / 2

		var j4 = %UR3e/b/mk_0/j0/mk_1/j1/mk_2/j2/mk_3/j3/mk_4 as Marker3D
		j4.rotation.z = - q_actual[4]

		var j5 = %UR3e/b/mk_0/j0/mk_1/j1/mk_2/j2/mk_3/j3/mk_4/j4/mk_5 as Marker3D
		j5.rotation.y = - q_actual[5]
