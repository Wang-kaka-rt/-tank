import React, { useEffect, useState } from 'react';
import { App as AntdApp, Button, Card, Flex, Input, Layout, Menu, Row, Col, Select, Segmented, Space, Tag, Typography, theme } from 'antd';
import { CarOutlined, RadarChartOutlined, EyeOutlined, DragOutlined, RobotOutlined, MessageOutlined } from '@ant-design/icons';
import { ConnectionManager } from '../components/ConnectionManager';
import { JoystickControl } from '../components/JoystickControl';
import { LidarControl } from './LidarControl';
import { VisualControl } from './VisualControl';
import { HandControl } from './HandControl';
import { Topic } from 'roslib';
import { useRos } from '../hooks/useRos';

const { Header, Content, Sider } = Layout;
const { Title, Text } = Typography;
const { TextArea } = Input;

const toolSpec = [
  {
    name: 'move',
    description: '发布底盘速度到 cmd_vel_topic 并持续 duration 秒。x/y 为线速度(m/s)，z 为角速度(rad/s)。',
    parameters: { x: 'float', y: 'float', z: 'float', duration: 'float' },
  },
  { name: 'stop', description: '立即停止底盘。', parameters: {} },
  {
    name: 'set_motor_rps',
    description: '直接设置 1~4 号电机转速(rps)。',
    parameters: { m1: 'float', m2: 'float', m3: 'float', m4: 'float' },
  },
  {
    name: 'set_servo_pwm',
    description: '设置 PWM 舵机位置。id 为舵机ID(1~4)，position 为脉宽(例如 1500)。',
    parameters: { id: 'int', position: 'int', duration: 'float' },
  },
  {
    name: 'set_buzzer',
    description: '控制蜂鸣器。freq(Hz), on_time/off_time(秒), repeat(次数)。',
    parameters: { freq: 'int', on_time: 'float', off_time: 'float', repeat: 'int' },
  },
  {
    name: 'set_led',
    description: '控制板载 LED 闪烁。id 为灯号。',
    parameters: { id: 'int', on_time: 'float', off_time: 'float', repeat: 'int' },
  },
  { name: 'set_rgb', description: '设置 RGB 灯珠颜色。', parameters: { pixels: 'list' } },
  { name: 'get_status', description: '读取当前电池等状态。', parameters: {} },
];

const defaultSystemPrompt = `你是一个机器人控制智能体。你只能从给定工具中选择一个或多个动作。必须输出JSON，不要输出多余文字。格式如下：{"actions":[{"tool":"move","args":{"x":0.2,"y":0.0,"z":0.0,"duration":1.0}},{"tool":"stop","args":{}}],"response":"简短回复"}工具列表如下：${JSON.stringify(
  toolSpec,
)}`;

type StringMessage = {
  data: string;
};

const LLMControlPanel: React.FC = () => {
  const { message } = AntdApp.useApp();
  const { ros, isConnected } = useRos();
  const [mode, setMode] = useState<'ros' | 'ollama'>('ollama');
  const [inputText, setInputText] = useState('');
  const [inputTopic, setInputTopic] = useState('/llm_agent/input');
  const [actionTopic, setActionTopic] = useState('/llm_agent/action');
  const [responseTopic, setResponseTopic] = useState('/llm_agent/response');
  const [ollamaBaseUrl, setOllamaBaseUrl] = useState('http://localhost:11434');
  const [ollamaModel, setOllamaModel] = useState('qwen3-vl:30b');
  const [ollamaApiMode, setOllamaApiMode] = useState<'ollama' | 'openai'>('ollama');
  const [apiKey, setApiKey] = useState('');
  const [systemPrompt, setSystemPrompt] = useState(defaultSystemPrompt);
  const [lastResponse, setLastResponse] = useState<string | null>(null);
  const [responseHistory, setResponseHistory] = useState<string[]>([]);
  const [lastLlmContent, setLastLlmContent] = useState<string>('');
  const [lastRosResponseAt, setLastRosResponseAt] = useState<number | null>(null);
  const [ollamaLatencyMs, setOllamaLatencyMs] = useState<number | null>(null);
  const [isSending, setIsSending] = useState(false);

  useEffect(() => {
    if (!ros || !isConnected || !responseTopic) return;
    const responseSub = new Topic<StringMessage>({
      ros,
      name: responseTopic,
      messageType: 'std_msgs/msg/String',
    });
    responseSub.subscribe((msg) => {
      setLastResponse(msg.data);
      setResponseHistory((prev) => [msg.data, ...prev].slice(0, 6));
      setLastRosResponseAt(Date.now());
    });
    return () => {
      responseSub.unsubscribe();
    };
  }, [ros, isConnected, responseTopic]);

  useEffect(() => {
    if (lastResponse) {
      setLastRosResponseAt(Date.now());
    }
  }, [lastResponse]);

  const publishString = (topicName: string, data: string) => {
    if (!ros || !isConnected) {
      message.error('ROS 未连接');
      return false;
    }
    const topic = new Topic<StringMessage>({
      ros,
      name: topicName,
      messageType: 'std_msgs/msg/String',
    });
    topic.publish({ data });
    return true;
  };

  const buildEndpoint = (baseUrl: string, apiMode: 'ollama' | 'openai') => {
    let url = baseUrl.trim();
    if (!url) return '';
    if (apiMode === 'openai') {
      if (url.endsWith('/v1/chat/completions')) return url;
      if (url.endsWith('/')) url = url.slice(0, -1);
      return `${url}/v1/chat/completions`;
    }
    if (url.endsWith('/api/chat')) return url;
    if (url.endsWith('/')) url = url.slice(0, -1);
    return `${url}/api/chat`;
  };

  const extractJsonLike = (content: string) => {
    const start = content.indexOf('{');
    const end = content.lastIndexOf('}');
    if (start === -1 || end === -1 || end <= start) return content.trim();
    return content.slice(start, end + 1);
  };

  const formatTime = (value: number | null) =>
    value === null || Number.isNaN(value) ? '—' : new Date(value).toLocaleTimeString();

  const handleSendToRos = () => {
    if (!inputText.trim()) {
      message.warning('请输入指令');
      return;
    }
    const ok = publishString(inputTopic, inputText.trim());
    if (ok) {
      message.success('已发送到 ROS LLM');
    }
  };

  const handleSendViaOllama = async () => {
    if (!inputText.trim()) {
      message.warning('请输入指令');
      return;
    }
    const endpoint = buildEndpoint(ollamaBaseUrl, ollamaApiMode);
    if (!endpoint) {
      message.error('请填写 Ollama 地址');
      return;
    }
    const startedAt = Date.now();
    setOllamaLatencyMs(null);
    setIsSending(true);
    try {
      const payload =
        ollamaApiMode === 'openai'
          ? {
              model: ollamaModel,
              messages: [
                { role: 'system', content: systemPrompt },
                { role: 'user', content: inputText.trim() },
              ],
              temperature: 0.2,
            }
          : {
              model: ollamaModel,
              messages: [
                { role: 'system', content: systemPrompt },
                { role: 'user', content: inputText.trim() },
              ],
              stream: false,
            };

      const res = await fetch(endpoint, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          ...(ollamaApiMode === 'openai' && apiKey ? { Authorization: `Bearer ${apiKey}` } : {}),
        },
        body: JSON.stringify(payload),
      });

      if (!res.ok) {
        const text = await res.text();
        throw new Error(text || `HTTP ${res.status}`);
      }
      const data = await res.json();
      const content =
        ollamaApiMode === 'openai'
          ? data?.choices?.[0]?.message?.content ?? ''
          : data?.message?.content ?? '';
      if (!content) {
        throw new Error('未返回有效内容');
      }
      setLastLlmContent(content);
      setOllamaLatencyMs(Date.now() - startedAt);
      const toSend = extractJsonLike(content);
      const ok = publishString(actionTopic, toSend);
      if (ok) {
        message.success('已调用 Ollama 并发送动作');
      }
    } catch (error) {
      message.error(`调用失败: ${String(error)}`);
    } finally {
      setIsSending(false);
    }
  };

  const canSend = isConnected && (!!inputText.trim() || !!lastLlmContent);

  return (
    <div style={{ width: '100%', maxWidth: 1400, margin: '0 auto' }}>
      <Row gutter={[24, 24]} align="top">
        <Col xs={24} lg={16}>
          <Card title="LLM 控制">
            <Flex vertical gap={16}>
              <Flex gap={12} wrap>
                <Segmented
                  value={mode}
                  onChange={(val) => setMode(val as 'ros' | 'ollama')}
                  options={[
                    { label: 'ROS LLM', value: 'ros' },
                    { label: 'Ollama', value: 'ollama' },
                  ]}
                />
                <Tag color={isConnected ? 'green' : 'red'}>
                  {isConnected ? 'ROS 已连接' : 'ROS 未连接'}
                </Tag>
              </Flex>

              <Flex vertical gap={12}>
                <Text>ROS 话题</Text>
                <Space direction="vertical" size={8} style={{ width: '100%' }}>
                  <Input value={inputTopic} onChange={(e) => setInputTopic(e.target.value)} placeholder="/llm_agent/input" />
                  <Input value={actionTopic} onChange={(e) => setActionTopic(e.target.value)} placeholder="/llm_agent/action" />
                  <Input value={responseTopic} onChange={(e) => setResponseTopic(e.target.value)} placeholder="/llm_agent/response" />
                </Space>
              </Flex>

              {mode === 'ollama' && (
                <Flex vertical gap={12}>
                  <Text>Ollama 配置</Text>
                  <Space direction="vertical" size={8} style={{ width: '100%' }}>
                    <Input
                      value={ollamaBaseUrl}
                      onChange={(e) => setOllamaBaseUrl(e.target.value)}
                      placeholder="http://localhost:11434"
                    />
                    <Space.Compact style={{ width: '100%' }}>
                      <Input
                        value={ollamaModel}
                        onChange={(e) => setOllamaModel(e.target.value)}
                        placeholder="qwen3-vl:30b"
                      />
                      <Select
                        value={ollamaApiMode}
                        onChange={(val) => setOllamaApiMode(val)}
                        options={[
                          { label: 'Ollama /api/chat', value: 'ollama' },
                          { label: 'OpenAI /v1/chat/completions', value: 'openai' },
                        ]}
                        style={{ minWidth: 220 }}
                      />
                    </Space.Compact>
                    {ollamaApiMode === 'openai' && (
                      <Input.Password
                        value={apiKey}
                        onChange={(e) => setApiKey(e.target.value)}
                        placeholder="API Key (可选)"
                      />
                    )}
                    <TextArea
                      value={systemPrompt}
                      onChange={(e) => setSystemPrompt(e.target.value)}
                      rows={6}
                      placeholder="系统提示词"
                    />
                  </Space>
                </Flex>
              )}

              <Flex vertical gap={10}>
                <Text>指令输入</Text>
                <TextArea
                  value={inputText}
                  onChange={(e) => setInputText(e.target.value)}
                  rows={4}
                  placeholder="例如：向前走1秒后停止"
                />
                <Space wrap>
                  {mode === 'ros' ? (
                    <Button type="primary" onClick={handleSendToRos} disabled={!canSend}>
                      发送到 ROS LLM
                    </Button>
                  ) : (
                    <Button type="primary" onClick={handleSendViaOllama} disabled={!canSend} loading={isSending}>
                      调用 Ollama 并执行
                    </Button>
                  )}
                </Space>
              </Flex>
            </Flex>
          </Card>
        </Col>
        <Col xs={24} lg={8}>
          <Card title="响应与日志">
            <Flex vertical gap={12}>
              <Flex vertical gap={4}>
                <Text type="secondary">Ollama 原始输出耗时：{ollamaLatencyMs === null ? '—' : `${ollamaLatencyMs} ms`}</Text>
                <Text type="secondary">ROS 响应时间：{formatTime(lastRosResponseAt)}</Text>
              </Flex>
              <Text type="secondary">/llm_agent/response 最新输出</Text>
              <TextArea value={lastResponse ?? ''} rows={4} readOnly />
              <Text type="secondary">Ollama 原始输出</Text>
              <TextArea value={lastLlmContent} rows={6} readOnly />
              <Text type="secondary">最近消息</Text>
              <TextArea value={responseHistory.join('\n\n')} rows={6} readOnly />
            </Flex>
          </Card>
        </Col>
      </Row>
    </div>
  );
};

export const Dashboard: React.FC = () => {
  const [selectedKey, setSelectedKey] = useState('chassis');
  const {
    token: { colorBgContainer, colorBgLayout, borderRadiusLG },
  } = theme.useToken();

  const renderContent = () => {
    switch (selectedKey) {
      case 'chassis':
        return (
          <Row className="chassis-grid" gutter={[16, 16]} justify="center" align="top">
            <Col xs={24} lg={14}>
              <JoystickControl />
            </Col>
          </Row>
        );
      case 'lidar':
        return <LidarControl />;
      case 'visual':
        return <VisualControl />;
      case 'hand':
        return <HandControl />;
      case 'llm':
        return <LLMControlPanel />;
      default:
        return <JoystickControl />;
    }
  };

  return (
    <Layout className="app-shell" style={{ minHeight: '100vh', background: colorBgLayout }}>
      <Sider className="app-sider" width={260} breakpoint="lg" collapsedWidth="0">
        <div className="app-logo">
          <RobotOutlined style={{ fontSize: 24, color: '#fff' }} />
          <span style={{ color: '#fff', fontSize: 18, fontWeight: 'bold' }}>Web Control</span>
        </div>
        <div className="app-sider-connection">
          <ConnectionManager variant="sidebar" />
        </div>
        <Menu
          theme="dark"
          mode="inline"
          defaultSelectedKeys={['chassis']}
          selectedKeys={[selectedKey]}
          onClick={(e) => setSelectedKey(e.key)}
          items={[
            { key: 'chassis', icon: <CarOutlined />, label: 'Chassis & Connection' },
            { key: 'lidar', icon: <RadarChartOutlined />, label: 'Lidar Control' },
            { key: 'visual', icon: <EyeOutlined />, label: 'Visual Tracking' },
            { key: 'hand', icon: <DragOutlined />, label: 'Hand Interaction' },
            { key: 'llm', icon: <MessageOutlined />, label: 'LLM Control' },
          ]}
        />
      </Sider>
      <Layout>
        <Header className="app-header" style={{ background: colorBgContainer }}>
          <Title level={4} style={{ margin: 0 }}>
            {selectedKey === 'chassis' && 'Chassis & Connection'}
            {selectedKey === 'lidar' && 'Lidar Control'}
            {selectedKey === 'visual' && 'Visual Tracking'}
            {selectedKey === 'hand' && 'Hand Interaction'}
            {selectedKey === 'llm' && 'LLM Control'}
          </Title>
        </Header>
        <Content className="app-content">
          <div
            className="app-surface"
            style={{
              background: colorBgContainer,
              borderRadius: borderRadiusLG,
            }}
          >
            {renderContent()}
          </div>
        </Content>
      </Layout>
    </Layout>
  );
};
