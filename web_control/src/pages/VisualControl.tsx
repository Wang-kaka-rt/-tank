import React, { useMemo, useState } from 'react';
import { App as AntdApp, Card, Tabs, Row, Col, Typography, Flex } from 'antd';
import { ServiceButton } from '../components/ServiceButton';
import { VideoDisplay } from '../components/VideoDisplay';
import { Service } from 'roslib';
import { useRos } from '../hooks/useRos';

const { Text } = Typography;

type SetPointRequest = {
  data: {
    x: number;
    y: number;
  };
};

type BasicServiceResponse = {
  success?: boolean;
  message?: string;
} & Record<string, unknown>;

export const VisualControl: React.FC = () => {
  const { message } = AntdApp.useApp();
  const { ros } = useRos();
  const [activeTab, setActiveTab] = useState('line_following');
  const setPointServiceType = useMemo(() => 'interfaces/srv/SetPoint', []);

  const handleColorPick = (x: number, y: number) => {
    if (!ros) return;
    
    const serviceName = activeTab === 'line_following' 
      ? '/line_following/set_target_color' 
      : '/object_tracking/set_target_color';
      
    const service = new Service<SetPointRequest, BasicServiceResponse>({
      ros,
      name: serviceName,
      serviceType: setPointServiceType,
    });

    service.callService({ data: { x, y } }, (result) => {
      if (result?.success !== false) {
        message.success(`Target color set at (${x.toFixed(2)}, ${y.toFixed(2)})`);
      } else {
        message.error(`Failed to set target color`);
      }
    });
  };

  const renderControls = (prefix: string, name: string) => (
    <Flex vertical gap={12} style={{ width: '100%' }}>
      <Flex wrap gap={12}>
        <ServiceButton 
          serviceName={`${prefix}/enter`} 
          serviceType="std_srvs/Trigger" 
          buttonText={`Enter ${name}`} 
          type="primary"
        />
        <ServiceButton 
          serviceName={`${prefix}/exit`} 
          serviceType="std_srvs/Trigger" 
          buttonText={`Exit ${name}`} 
          danger
        />
      </Flex>
      
      <Flex wrap gap={12}>
        <ServiceButton 
          serviceName={`${prefix}/set_running`} 
          serviceType="std_srvs/SetBool" 
          requestData={{ data: true }}
          buttonText="Start Running" 
        />
        <ServiceButton 
          serviceName={`${prefix}/set_running`} 
          serviceType="std_srvs/SetBool" 
          requestData={{ data: false }}
          buttonText="Stop Running" 
        />
      </Flex>

      <Text type="secondary">Click on the video stream to set target color.</Text>
    </Flex>
  );

  return (
    <div style={{ width: '100%', maxWidth: 1400, margin: '0 auto' }}>
      <Row gutter={[24, 24]} align="top">
        <Col xs={24} lg={14}>
          <VideoDisplay onClick={handleColorPick} />
        </Col>
        <Col xs={24} lg={10}>
          <Card style={{ height: '100%' }}>
            <Tabs 
              activeKey={activeTab} 
              onChange={setActiveTab}
              items={[
                {
                  key: 'line_following',
                  label: 'Line Following',
                  children: renderControls('/line_following', 'Line Following')
                },
                {
                  key: 'object_tracking',
                  label: 'Object Tracking',
                  children: renderControls('/object_tracking', 'Object Tracking')
                }
              ]}
            />
          </Card>
        </Col>
      </Row>
    </div>
  );
};
