import React, { useEffect, useMemo, useRef, useState } from 'react';
import { Joystick } from 'react-joystick-component';
import { Topic } from 'roslib';
import { useRos } from '../hooks/useRos';
import { Button, Card, Col, Divider, Flex, Row, Segmented, Slider, Space, Tag, Typography } from 'antd';

type TwistLike = {
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
};

type JoystickMoveEvent = {
  x?: number | null;
  y?: number | null;
};

const createZeroTwist = (): TwistLike => ({
  linear: { x: 0, y: 0, z: 0 },
  angular: { x: 0, y: 0, z: 0 },
});

export const JoystickControl: React.FC = () => {
  const { ros, isConnected } = useRos();
  const cmdVel = useRef<Topic<TwistLike> | null>(null);
  const [cmdVelTopic, setCmdVelTopic] = useState<'/cmd_vel' | '/controller/cmd_vel'>('/controller/cmd_vel');
  const messageType = useMemo(() => 'geometry_msgs/msg/Twist', []);
  const publishTimerRef = useRef<number | null>(null);
  const lastTwistRef = useRef<TwistLike>(createZeroTwist());
  const [lastTwist, setLastTwist] = useState<TwistLike>(createZeroTwist);
  const lastPublishAtRef = useRef<number>(0);
  const [lastPublishAt, setLastPublishAt] = useState<number | null>(null);
  const [linearMax, setLinearMax] = useState<number>(0.2);
  const [angularMax, setAngularMax] = useState<number>(0.5);

  const odomRawRef = useRef<Topic<Record<string, unknown>> | null>(null);
  const [lastOdomAt, setLastOdomAt] = useState<number | null>(null);
  const cmdVelEchoRef = useRef<Topic<TwistLike> | null>(null);
  const [lastCmdEchoAt, setLastCmdEchoAt] = useState<number | null>(null);
  const [lastEchoTwist, setLastEchoTwist] = useState<TwistLike | null>(null);
  const [now, setNow] = useState<number>(() => Date.now());

  useEffect(() => {
    if (ros && isConnected) {
      const topic = new Topic<TwistLike>({
        ros,
        name: cmdVelTopic,
        messageType,
      });
      topic.advertise();
      cmdVel.current = topic;
      return () => {
        if (publishTimerRef.current) {
          window.clearInterval(publishTimerRef.current);
          publishTimerRef.current = null;
        }
        topic.unadvertise();
      };
    } else {
      cmdVel.current = null;
    }
  }, [ros, isConnected, cmdVelTopic, messageType]);

  useEffect(() => {
    if (!ros || !isConnected) {
      if (publishTimerRef.current) {
        window.clearInterval(publishTimerRef.current);
        publishTimerRef.current = null;
      }
      if (odomRawRef.current) {
        odomRawRef.current.unsubscribe();
        odomRawRef.current = null;
      }
      if (cmdVelEchoRef.current) {
        cmdVelEchoRef.current.unsubscribe();
        cmdVelEchoRef.current = null;
      }
      return;
    }

    const odomRaw = new Topic<Record<string, unknown>>({
      ros,
      name: '/odom_raw',
      messageType: 'nav_msgs/msg/Odometry',
    });
    odomRawRef.current = odomRaw;
    odomRaw.subscribe(() => {
      setLastOdomAt(Date.now());
    });

    const cmdEcho = new Topic<TwistLike>({
      ros,
      name: cmdVelTopic,
      messageType,
    });
    cmdVelEchoRef.current = cmdEcho;
    cmdEcho.subscribe((msg) => {
      setLastCmdEchoAt(Date.now());
      setLastEchoTwist(msg);
    });

    return () => {
      if (publishTimerRef.current) {
        window.clearInterval(publishTimerRef.current);
        publishTimerRef.current = null;
      }
      if (odomRawRef.current) {
        odomRawRef.current.unsubscribe();
        odomRawRef.current = null;
      }
      if (cmdVelEchoRef.current) {
        cmdVelEchoRef.current.unsubscribe();
        cmdVelEchoRef.current = null;
      }
    };
  }, [ros, isConnected, cmdVelTopic, messageType]);

  useEffect(() => {
    if (!isConnected) return;
    const timer = window.setInterval(() => setNow(Date.now()), 250);
    return () => window.clearInterval(timer);
  }, [isConnected]);

  const publishCurrentTwist = () => {
    if (!cmdVel.current) return;
    cmdVel.current.publish(lastTwistRef.current);
    const now = Date.now();
    lastPublishAtRef.current = now;
    setLastPublishAt(now);
  };

  const ensurePublishLoop = () => {
    if (publishTimerRef.current) return;
    publishTimerRef.current = window.setInterval(() => {
      publishCurrentTwist();
    }, 50);
  };

  const handleMove = (event: JoystickMoveEvent) => {
    if (!cmdVel.current) return;

    // Mapping joystick x/y to linear/angular velocity
    // Assuming x is left/right (angular z) and y is up/down (linear x)
    // Adjust scaling factors as needed
    const linearSpeed = (((event.y ?? 0) || 0) / 50.0) * linearMax;
    const angularSpeed = -(((event.x ?? 0) || 0) / 50.0) * angularMax;

    lastTwistRef.current = {
      linear: {
        x: linearSpeed,
        y: 0.0,
        z: 0.0,
      },
      angular: {
        x: 0.0,
        y: 0.0,
        z: angularSpeed,
      },
    };
    setLastTwist(lastTwistRef.current);

    ensurePublishLoop();
    publishCurrentTwist();
  };

  const handleStop = () => {
    if (!cmdVel.current) return;

    lastTwistRef.current = {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    };
    setLastTwist(lastTwistRef.current);
    publishCurrentTwist();

    if (publishTimerRef.current) {
      window.clearInterval(publishTimerRef.current);
      publishTimerRef.current = null;
    }
  };

  const odomAlive = lastOdomAt ? now - lastOdomAt < 1000 : false;
  const cmdAlive = lastPublishAt ? now - lastPublishAt < 1000 : false;
  const cmdEchoAlive = lastCmdEchoAt ? now - lastCmdEchoAt < 1000 : false;

  const setTwistAndRun = (linearX: number, angularZ: number) => {
    lastTwistRef.current = {
      linear: { x: linearX, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angularZ },
    };
    setLastTwist(lastTwistRef.current);
    ensurePublishLoop();
    publishCurrentTwist();
  };

  return (
    <Card className="chassis-card" title="Chassis Control" style={{ width: '100%' }}>
      <div className="chassis-card__toolbar">
        <Segmented
          value={cmdVelTopic}
          onChange={(val) => setCmdVelTopic(val as '/cmd_vel' | '/controller/cmd_vel')}
          options={[
            { label: '/controller/cmd_vel', value: '/controller/cmd_vel' },
            { label: '/cmd_vel', value: '/cmd_vel' },
          ]}
        />
        <Space size={6} wrap>
          <Tag color={isConnected ? 'green' : 'red'}>{isConnected ? 'WS Connected' : 'WS Disconnected'}</Tag>
          <Tag color={odomAlive ? 'green' : 'orange'}>{odomAlive ? 'odom_raw OK' : 'odom_raw N/A'}</Tag>
          <Tag color={cmdAlive ? 'blue' : 'default'}>{cmdAlive ? 'cmd_vel publishing' : 'cmd_vel idle'}</Tag>
          <Tag color={cmdEchoAlive ? 'green' : 'orange'}>{cmdEchoAlive ? 'cmd_vel echo OK' : 'cmd_vel echo N/A'}</Tag>
        </Space>
      </div>
      {(isConnected && !odomAlive) || (isConnected && odomAlive && !cmdAlive) ? (
        <div className="chassis-card__hints">
          {!odomAlive && (
            <Typography.Text type="secondary">
              收不到 /odom_raw 一般表示你连到的 ROS 图里没启动 controller（或 rosbridge 在另一台机器/Domain）。
            </Typography.Text>
          )}
          {isConnected && odomAlive && !cmdAlive && (
            <Typography.Text type="secondary">
              现在显示 cmd_vel idle，说明你的输入事件没有触发发送。可以用下面的“方向按钮”先验证是否能动。
            </Typography.Text>
          )}
        </div>
      ) : null}
      <Row gutter={[20, 20]} align="middle" className="chassis-card__content">
        <Col xs={24} lg={14}>
          <Space className="chassis-controls" orientation="vertical" size={12}>
            <div className="chassis-section">
              <Typography.Text className="chassis-section__title">速度设置</Typography.Text>
              <Flex align="center" gap={12} className="chassis-slider">
                <Typography.Text style={{ width: 72 }} type="secondary">
                  线速度
                </Typography.Text>
                <Slider
                  min={0.05}
                  max={0.6}
                  step={0.01}
                  value={linearMax}
                  onChange={(v) => setLinearMax(v as number)}
                />
                <Typography.Text className="chassis-value">{linearMax.toFixed(2)}</Typography.Text>
              </Flex>
              <Flex align="center" gap={12} className="chassis-slider">
                <Typography.Text style={{ width: 72 }} type="secondary">
                  角速度
                </Typography.Text>
                <Slider
                  min={0.1}
                  max={2.0}
                  step={0.05}
                  value={angularMax}
                  onChange={(v) => setAngularMax(v as number)}
                />
                <Typography.Text className="chassis-value">{angularMax.toFixed(2)}</Typography.Text>
              </Flex>
            </div>
            <Divider className="chassis-divider" />
            <div className="chassis-section">
              <Typography.Text className="chassis-section__title">方向控制</Typography.Text>
              <Flex gap={8} wrap className="chassis-actions">
              <Button
                type="primary"
                onMouseDown={() => setTwistAndRun(linearMax, 0)}
                onMouseUp={handleStop}
                onMouseLeave={handleStop}
                onTouchStart={() => setTwistAndRun(linearMax, 0)}
                onTouchEnd={handleStop}
              >
                前进
              </Button>
              <Button
                onMouseDown={() => setTwistAndRun(-linearMax, 0)}
                onMouseUp={handleStop}
                onMouseLeave={handleStop}
                onTouchStart={() => setTwistAndRun(-linearMax, 0)}
                onTouchEnd={handleStop}
              >
                后退
              </Button>
              <Button
                onMouseDown={() => setTwistAndRun(0, angularMax)}
                onMouseUp={handleStop}
                onMouseLeave={handleStop}
                onTouchStart={() => setTwistAndRun(0, angularMax)}
                onTouchEnd={handleStop}
              >
                左转
              </Button>
              <Button
                onMouseDown={() => setTwistAndRun(0, -angularMax)}
                onMouseUp={handleStop}
                onMouseLeave={handleStop}
                onTouchStart={() => setTwistAndRun(0, -angularMax)}
                onTouchEnd={handleStop}
              >
                右转
              </Button>
              <Button danger onClick={handleStop}>
                停止
              </Button>
              </Flex>
            </div>
            <div className="chassis-metrics">
              <Typography.Text type="secondary">
                发送值：linear.x={lastTwist.linear.x.toFixed(2)}，angular.z={lastTwist.angular.z.toFixed(2)}
                {lastEchoTwist
                  ? ` ｜回显：linear.x=${lastEchoTwist.linear.x.toFixed(2)}，angular.z=${lastEchoTwist.angular.z.toFixed(2)}`
                  : ''}
              </Typography.Text>
            </div>
          </Space>
        </Col>
        <Col xs={24} lg={10}>
          <div className="chassis-joystick-wrap">
            <Joystick
              size={150}
              sticky={false}
              baseColor="#EEEEEE"
              stickColor="#1890ff"
              move={handleMove}
              stop={handleStop}
            />
          </div>
        </Col>
      </Row>
    </Card>
  );
};
