import React from 'react';
import { Card, Tabs, Row, Col, Flex } from 'antd';
import { ServiceButton } from '../components/ServiceButton';
import { VideoDisplay } from '../components/VideoDisplay';

export const HandControl: React.FC = () => {
  return (
    <div style={{ width: '100%', maxWidth: 1400, margin: '0 auto' }}>
      <Row gutter={[24, 24]} align="top">
        <Col xs={24} lg={14}>
          <VideoDisplay topic="/depth_cam/rgb/image_raw" />
        </Col>
        <Col xs={24} lg={10}>
          <Card style={{ height: '100%' }}>
            <Tabs 
              items={[
                {
                  key: 'gesture',
                  label: 'Hand Gesture',
                  children: (
                    <Flex vertical gap={12}>
                      <Flex wrap gap={12}>
                      <ServiceButton 
                        serviceName="/hand_gesture/enter" 
                        serviceType="std_srvs/Trigger" 
                        buttonText="Enter Gesture Mode" 
                        type="primary"
                      />
                      <ServiceButton 
                        serviceName="/hand_gesture/exit" 
                        serviceType="std_srvs/Trigger" 
                        buttonText="Exit Gesture Mode" 
                        danger
                      />
                      </Flex>
                      <Flex wrap gap={12}>
                      <ServiceButton 
                        serviceName="/hand_gesture/set_running" 
                        serviceType="std_srvs/SetBool" 
                        requestData={{ data: true }}
                        buttonText="Start Running" 
                      />
                      <ServiceButton 
                        serviceName="/hand_gesture/set_running" 
                        serviceType="std_srvs/SetBool" 
                        requestData={{ data: false }}
                        buttonText="Stop Running" 
                      />
                      </Flex>
                    </Flex>
                  )
                },
                {
                  key: 'trajectory',
                  label: 'Hand Trajectory',
                  children: (
                    <Flex vertical gap={12}>
                      <Flex wrap gap={12}>
                      <ServiceButton 
                        serviceName="/hand_trajectory/enter" 
                        serviceType="std_srvs/Trigger" 
                        buttonText="Enter Trajectory Mode" 
                        type="primary"
                      />
                      <ServiceButton 
                        serviceName="/hand_trajectory/exit" 
                        serviceType="std_srvs/Trigger" 
                        buttonText="Exit Trajectory Mode" 
                        danger
                      />
                      </Flex>
                      <Flex wrap gap={12}>
                      <ServiceButton 
                        serviceName="/hand_trajectory/start" 
                        serviceType="std_srvs/Trigger" 
                        buttonText="Start Trajectory" 
                      />
                      <ServiceButton 
                        serviceName="/hand_trajectory/stop" 
                        serviceType="std_srvs/Trigger" 
                        buttonText="Stop Trajectory" 
                      />
                      </Flex>
                    </Flex>
                  )
                }
              ]}
            />
          </Card>
        </Col>
      </Row>
    </div>
  );
};
