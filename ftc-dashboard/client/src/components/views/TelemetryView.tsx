import { useEffect, useState } from 'react';
import { useSelector } from 'react-redux';

import BaseView, {
  BaseViewHeading,
  BaseViewBody,
  BaseViewProps,
  BaseViewHeadingProps,
} from './BaseView';
import { RootState } from '@/store/reducers';

type TelemetryViewProps = BaseViewProps & BaseViewHeadingProps;

const TelemetryView = ({
  isDraggable = false,
  isUnlocked = false,
}: TelemetryViewProps) => {
  const [log, setLog] = useState<string[]>([]);
  const [data, setData] = useState<{ [key: string]: string }>({});

  const packets = useSelector((state: RootState) => state.telemetry);
  useEffect(() => {
    if (packets.length === 0) {
      setLog([]);
      setData({});
      return;
    }

    setLog((prevLog) =>
      packets.reduce(
        (acc, { log: newLog }) => (newLog.length === 0 ? acc : newLog),
        prevLog,
      ),
    );

    setData((prevData) =>
      packets.reduce(
        (acc, { data: newData }) =>
          Object.keys(newData).reduce(
            (acc, k) => ({ ...acc, [k]: newData[k] }),
            acc,
          ),
        prevData,
      ),
    );
  }, [packets]);

  const telemetryLines = Object.keys(data).map((key) => (
    <span
      key={key}
      dangerouslySetInnerHTML={{ __html: `${key}: ${data[key]}<br />` }}
    />
  ));

  const telemetryLog = log.map((line, i) => (
    <span key={i} dangerouslySetInnerHTML={{ __html: `${line}<br />` }} />
  ));

  return (
    <BaseView isUnlocked={isUnlocked}>
      <BaseViewHeading
        isDraggable={isDraggable}
        className="flex items-center justify-between"
      >
        <span>Telemetry</span>
        <a
          className="rounded bg-slate-100 px-2 py-1 text-sm text-slate-900 hover:bg-slate-200 dark:bg-slate-800 dark:text-slate-100 dark:hover:bg-slate-700"
          href="/dash/hephatelemetry.log"
          download="hephatelemetry.log"
        >
          Download Log
        </a>
      </BaseViewHeading>
      <BaseViewBody>
        <p>{telemetryLines}</p>
        <p>{telemetryLog}</p>
      </BaseViewBody>
    </BaseView>
  );
};

export default TelemetryView;
