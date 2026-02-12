from motion.core import RobotControl


def main():
	robot = RobotControl(ip = '10.20.6.254')

	if robot.connect():
		print('Робот подключен')

		if robot.engage():
			print('Двигатели робота включены') 

			if robot.manualCartMode():
				print('Включен режим движения в декартовых координатах')
			else:
				print('Вознкла ошибка')
				return

			velocity = [0.1, 0, 0, 0, 0, 0]

			robot.setCartesianVelocity(velocity)

			if robot.disengage():
				print('Моторы робота выключены')
			else:
				print('Возникла ошибка')
			return

if __name__ == '__main__':
	main()