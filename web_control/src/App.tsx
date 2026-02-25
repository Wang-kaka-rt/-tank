import React from 'react';
import { App as AntdApp, ConfigProvider, theme } from 'antd';
import { RosProvider } from './contexts/RosContext';
import { Dashboard } from './pages/Dashboard';
import 'antd/dist/reset.css';

const App: React.FC = () => {
  return (
    <ConfigProvider
      theme={{
        algorithm: theme.darkAlgorithm,
        token: {
          colorPrimary: '#1677ff',
        },
      }}
    >
      <AntdApp>
        <RosProvider>
          <Dashboard />
        </RosProvider>
      </AntdApp>
    </ConfigProvider>
  );
};

export default App;
