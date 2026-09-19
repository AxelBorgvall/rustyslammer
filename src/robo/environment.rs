use crate::robo::Message;



pub trait Environment: Send {
    fn spawn(&self, lidar_out: Message<String>);
}