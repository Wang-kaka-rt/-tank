import React from 'react';
import { Card, Divider, Flex } from 'antd';
import { ServiceButton } from '../components/ServiceButton';

export const LidarControl: React.FC = () => {
  return (
    <div style={{ width: '100%', maxWidth: 980, margin: '0 auto' }}>
      <Card title="Lidar App Control">
        <Flex wrap gap={12}>
          <ServiceButton 
            serviceName="/lidar_app/enter" 
            serviceType="std_srvs/Trigger" 
            buttonText="Enter Lidar App" 
            type="primary"
          />
          <ServiceButton 
            serviceName="/lidar_app/exit" 
            serviceType="std_srvs/Trigger" 
            buttonText="Exit Lidar App" 
            danger
          />
        </Flex>
        
        <Divider titlePlacement="left">Modes</Divider>
        <Flex wrap gap={12}>
          <ServiceButton 
            serviceName="/lidar_app/set_running" 
            serviceType="interfaces/SetInt64" 
            requestData={{ data: 0 }}
            buttonText="Stop Mode (0)" 
          />
          <ServiceButton 
            serviceName="/lidar_app/set_running" 
            serviceType="interfaces/SetInt64" 
            requestData={{ data: 1 }}
            buttonText="Obstacle Avoidance (1)" 
          />
          <ServiceButton 
            serviceName="/lidar_app/set_running" 
            serviceType="interfaces/SetInt64" 
            requestData={{ data: 2 }}
            buttonText="Lidar Following (2)" 
          />
          <ServiceButton 
            serviceName="/lidar_app/set_running" 
            serviceType="interfaces/SetInt64" 
            requestData={{ data: 3 }}
            buttonText="Guarding (3)" 
          />
        </Flex>
      </Card>
    </div>
  );
};
