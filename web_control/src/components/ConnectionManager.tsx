import React, { useState, useEffect } from 'react';
import { Input, Button, Card, Space, Typography, Badge, Flex } from 'antd';
import { useRos } from '../hooks/useRos';
import { Wifi } from 'lucide-react';

const { Text } = Typography;

type ConnectionManagerProps = {
  variant?: 'default' | 'sidebar';
};

export const ConnectionManager: React.FC<ConnectionManagerProps> = ({ variant = 'default' }) => {
  const { isConnected, connect, disconnect, url } = useRos();
  const [inputUrl, setInputUrl] = useState(url);

  useEffect(() => {
    setInputUrl(url);
  }, [url]);

  const handleConnect = () => {
    connect(inputUrl);
  };

  return (
    <Card 
      title={variant === 'sidebar' ? 'Connection' : 'Connection Settings'}
      extra={
        isConnected ? (
          <Badge status="success" text={<Text type="success">Connected</Text>} />
        ) : (
          <Badge status="error" text={<Text type="secondary">Disconnected</Text>} />
        )
      }
      size={variant === 'sidebar' ? 'small' : undefined}
      className={variant === 'sidebar' ? 'connection-manager connection-manager--sidebar' : 'connection-manager'}
      style={{ width: '100%' }}
    >
      <Flex vertical gap={12} style={{ width: '100%' }}>
        <Space.Compact style={{ width: '100%' }}>
          <Input 
            prefix={<Wifi size={16} />}
            value={inputUrl} 
            onChange={(e) => setInputUrl(e.target.value)} 
            placeholder="ws://localhost:9090" 
            onPressEnter={isConnected ? undefined : handleConnect}
            style={variant === 'sidebar' ? { flex: 1, minWidth: 0 } : undefined}
          />
          {isConnected ? (
            <Button type="primary" danger onClick={disconnect} style={variant === 'sidebar' ? { whiteSpace: 'nowrap' } : undefined}>
              Disconnect
            </Button>
          ) : (
            <Button type="primary" onClick={handleConnect} style={variant === 'sidebar' ? { whiteSpace: 'nowrap' } : undefined}>
              Connect
            </Button>
          )}
        </Space.Compact>
      </Flex>
    </Card>
  );
};
