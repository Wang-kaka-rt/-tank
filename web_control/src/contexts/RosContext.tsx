import React, { useEffect, useState } from 'react';
import type { ReactNode } from 'react';
import { Ros } from 'roslib';
import { App as AntdApp } from 'antd';
import { RosContext } from './RosContextCore';

export const RosProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const { message } = AntdApp.useApp();
  const [ros, setRos] = useState<Ros | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [url, setUrl] = useState<string>('ws://localhost:9090');

  useEffect(() => {
    return () => {
      if (ros) {
        ros.close();
      }
    };
  }, [ros]);

  const connect = (newUrl: string) => {
    if (ros) {
      ros.close();
    }

    const newRos = new Ros({
      url: newUrl,
    });

    newRos.on('connection', () => {
      console.log('Connected to websocket server.');
      setIsConnected(true);
      message.success('Connected to ROS Bridge');
    });

    newRos.on('error', (error) => {
      console.log('Error connecting to websocket server: ', error);
      setIsConnected(false);
      message.error('Connection Error');
    });

    newRos.on('close', () => {
      console.log('Connection to websocket server closed.');
      setIsConnected(false);
      message.warning('Connection Closed');
    });

    setRos(newRos);
    setUrl(newUrl);
  };

  const disconnect = () => {
    if (ros) {
      ros.close();
      setRos(null);
      setIsConnected(false);
    }
  };

  return (
    <RosContext.Provider value={{ ros, isConnected, url, connect, disconnect }}>
      {children}
    </RosContext.Provider>
  );
};
