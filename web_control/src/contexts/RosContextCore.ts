import { createContext } from 'react';
import type { Ros } from 'roslib';

export interface RosContextProps {
  ros: Ros | null;
  isConnected: boolean;
  url: string;
  connect: (url: string) => void;
  disconnect: () => void;
}

export const RosContext = createContext<RosContextProps | undefined>(undefined);

