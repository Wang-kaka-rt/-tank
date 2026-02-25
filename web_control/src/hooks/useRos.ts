import { useContext } from 'react';
import { RosContext } from '../contexts/RosContextCore';

export const useRos = () => {
  const context = useContext(RosContext);
  if (context === undefined) {
    throw new Error('useRos must be used within a RosProvider');
  }
  return context;
};
