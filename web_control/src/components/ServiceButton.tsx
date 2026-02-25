import React, { useMemo, useState } from 'react';
import { App as AntdApp, Button } from 'antd';
import { Service } from 'roslib';
import { useRos } from '../hooks/useRos';

type ServiceRequest = Record<string, unknown>;
type ServiceResponse = {
  success?: boolean;
  message?: string;
} & Record<string, unknown>;

interface ServiceButtonProps {
  serviceName: string;
  serviceType: string;
  requestData?: ServiceRequest;
  buttonText: string;
  onSuccess?: (response: ServiceResponse) => void;
  danger?: boolean;
  type?: "primary" | "default" | "dashed" | "link" | "text";
}

export const ServiceButton: React.FC<ServiceButtonProps> = ({
  serviceName,
  serviceType,
  requestData = {},
  buttonText,
  onSuccess,
  danger,
  type = "default"
}) => {
  const { message } = AntdApp.useApp();
  const { ros, isConnected } = useRos();
  const [loading, setLoading] = useState(false);
  const normalizedServiceType = useMemo(() => {
    if (!serviceType) return serviceType;
    if (serviceType.includes('/srv/')) return serviceType;
    const parts = serviceType.split('/');
    if (parts.length === 2) return `${parts[0]}/srv/${parts[1]}`;
    return serviceType;
  }, [serviceType]);

  const handleClick = () => {
    if (!ros || !isConnected) {
      message.error('Not connected to ROS');
      return;
    }

    setLoading(true);
    const service = new Service<ServiceRequest, ServiceResponse>({
      ros,
      name: serviceName,
      serviceType: normalizedServiceType,
    });

    service.callService(requestData, (result) => {
      setLoading(false);
      if (result?.success !== false) {
        message.success(`Called ${serviceName} successfully`);
        if (onSuccess) onSuccess(result);
      } else {
        message.error(`Service call failed: ${result?.message || 'Unknown error'}`);
      }
    }, (error) => {
      setLoading(false);
      message.error(`Failed to call service: ${error}`);
    });
  };

  return (
    <Button 
      type={type} 
      danger={danger} 
      loading={loading} 
      onClick={handleClick}
      disabled={!isConnected}
    >
      {buttonText}
    </Button>
  );
};
