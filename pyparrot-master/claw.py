from pyparrot.Minidrone import Mambo

mamboAddr = "D0:3A:5E:20:E6:21"

mambo = Mambo(mamboAddr, use_wifi=False)

print("trying to connect")
success = mambo.connect(num_retries=3)
print("connected: %s" % success)

if(sucess):
	try:
		mambo.ask_for_state_update()
		mambo.smart_sleep(1)

		print("taking off!")
		mambo.safe_takeoff(5)
		mambo.smart_sleep(1)

		print("open the claw")
		mambo.open_claw()
		mambo.smart_sleep(2)

		print("close the claw")
		mambo.close_claw()
		mambo.smart_sleep(2)

	except:
		print("cancelado")

	print("landing")
	mambo.safe_land(5)
	mambo.smart_sleep(1)

	print("disconnect")
	mambo.disconnect()