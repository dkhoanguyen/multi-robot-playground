from mrp_comms_msgs.srv import GetMembersInTeam

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(GetMembersInTeam,
                                       '/robot0/get_members_from_team',
                                       self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        print("Receiving request")
        print(request)
        response.member_name_list = ['robot0','robot1','robot2','robot3']
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
