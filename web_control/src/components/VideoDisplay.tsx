import React, { useMemo, useRef } from 'react';
import { Card, Spin, Flex, Typography } from 'antd';
import { useRos } from '../hooks/useRos';

const { Text } = Typography;

interface VideoDisplayProps {
  topic?: string;
  onClick?: (x: number, y: number) => void;
}

export const VideoDisplay: React.FC<VideoDisplayProps> = ({ topic = '/depth_cam/rgb/image_raw', onClick }) => {
  const { url, isConnected } = useRos();
  const imgRef = useRef<HTMLImageElement>(null);

  const streamUrl = useMemo(() => {
    if (!url) return '';
    try {
      const match = url.match(/ws:\/\/([^:]+):/);
      if (match && match[1]) {
        const ip = match[1];
        return `http://${ip}:8080/stream?topic=${topic}&type=mjpeg&quality=80`;
      }
      return '';
    } catch {
      return '';
    }
  }, [url, topic]);

  const handleClick = (e: React.MouseEvent<HTMLImageElement>) => {
    if (!onClick || !imgRef.current) return;

    const rect = imgRef.current.getBoundingClientRect();
    const x = (e.clientX - rect.left) / rect.width;
    const y = (e.clientY - rect.top) / rect.height;
    
    // Ensure coordinates are within 0-1 bounds
    const normalizedX = Math.max(0, Math.min(1, x));
    const normalizedY = Math.max(0, Math.min(1, y));

    onClick(normalizedX, normalizedY);
  };

  return (
    <Card
      className="video-card"
      title="Camera Feed"
      styles={{
        body: {
          padding: 0,
        },
      }}
    >
      <div
        style={{
          width: '100%',
          aspectRatio: '16 / 9',
          background: '#000',
          borderRadius: 8,
          overflow: 'hidden',
          minHeight: 260,
        }}
      >
        {isConnected && streamUrl ? (
          <img
            ref={imgRef}
            src={streamUrl}
            alt="ROS Video Stream"
            style={{
              width: '100%',
              height: '100%',
              objectFit: 'contain',
              cursor: onClick ? 'crosshair' : 'default',
            }}
            onClick={handleClick}
          />
        ) : (
          <Flex
            align="center"
            justify="center"
            vertical
            gap={8}
            style={{ width: '100%', height: '100%', color: 'rgba(255,255,255,0.85)' }}
          >
            {!isConnected ? (
              <Text style={{ color: 'rgba(255,255,255,0.85)' }}>Not Connected</Text>
            ) : (
              <Spin />
            )}
          </Flex>
        )}
      </div>
    </Card>
  );
};
