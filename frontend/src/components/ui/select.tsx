import React from "react";

interface SelectProps extends React.SelectHTMLAttributes<HTMLSelectElement> {
  children: React.ReactNode;
}

interface SelectItemProps extends React.OptionHTMLAttributes<HTMLOptionElement> {
  value: string;
  children: React.ReactNode;
}

interface SelectTriggerProps extends React.HTMLAttributes<HTMLDivElement> {
  children?: React.ReactNode;
}

interface SelectContentProps extends React.HTMLAttributes<HTMLDivElement> {
  children: React.ReactNode;
}

interface SelectValueProps {
  placeholder?: string;
}

const Select = ({ children, ...props }: SelectProps) => {
  return (
    <select {...props}>
      {children}
    </select>
  );
};

const SelectItem = ({ value, children, ...props }: SelectItemProps) => {
  return (
    <option value={value} {...props}>
      {children}
    </option>
  );
};

const SelectTrigger = ({ children, className, ...props }: SelectTriggerProps) => {
  const baseClasses = "flex h-10 w-full items-center justify-between rounded-md border border-input bg-background px-3 py-2 text-sm ring-offset-background placeholder:text-muted-foreground focus:outline-none focus:ring-2 focus:ring-ring focus:ring-offset-2 disabled:cursor-not-allowed disabled:opacity-50";

  const classes = `${baseClasses} ${className || ""}`;

  return (
    <div className={classes} {...props}>
      {children}
    </div>
  );
};

const SelectContent = ({ children, className, ...props }: SelectContentProps) => {
  const baseClasses = "relative z-50 min-w-[8rem] overflow-hidden rounded-md border bg-popover text-popover-foreground shadow-md data-[state=open]:animate-in data-[state=closed]:animate-out data-[state=closed]:fade-out-0 data-[state=open]:fade-in-0 data-[state=closed]:zoom-out-95 data-[state=open]:zoom-in-95";

  const classes = `${baseClasses} ${className || ""}`;

  return (
    <div className={classes} {...props}>
      {children}
    </div>
  );
};

const SelectValue = ({ placeholder }: SelectValueProps) => {
  return <>{placeholder}</>;
};

export {
  Select,
  SelectItem,
  SelectTrigger,
  SelectContent,
  SelectValue,
};