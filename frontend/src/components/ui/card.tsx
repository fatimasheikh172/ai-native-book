import React from "react";

interface CardProps extends React.HTMLAttributes<HTMLDivElement> {}

interface CardHeaderProps extends React.HTMLAttributes<HTMLDivElement> {}

interface CardTitleProps extends React.HTMLAttributes<HTMLHeadingElement> {}

interface CardDescriptionProps extends React.HTMLAttributes<HTMLParagraphElement> {}

interface CardContentProps extends React.HTMLAttributes<HTMLDivElement> {}

interface CardFooterProps extends React.HTMLAttributes<HTMLDivElement> {}

const Card = ({ className, ...props }: CardProps) => {
  const baseClasses = "rounded-lg border bg-card text-card-foreground shadow-sm";

  const classes = `${baseClasses} ${className || ""}`;

  return <div className={classes} {...props} />;
};

const CardHeader = ({ className, ...props }: CardHeaderProps) => {
  const baseClasses = "flex flex-col space-y-1.5 p-6";

  const classes = `${baseClasses} ${className || ""}`;

  return <div className={classes} {...props} />;
};

const CardTitle = ({ className, ...props }: CardTitleProps) => {
  const baseClasses = "text-2xl font-semibold leading-none tracking-tight";

  const classes = `${baseClasses} ${className || ""}`;

  return <h3 className={classes} {...props} />;
};

const CardDescription = ({ className, ...props }: CardDescriptionProps) => {
  const baseClasses = "text-sm text-muted-foreground";

  const classes = `${baseClasses} ${className || ""}`;

  return <p className={classes} {...props} />;
};

const CardContent = ({ className, ...props }: CardContentProps) => {
  const baseClasses = "p-6 pt-0";

  const classes = `${baseClasses} ${className || ""}`;

  return <div className={classes} {...props} />;
};

const CardFooter = ({ className, ...props }: CardFooterProps) => {
  const baseClasses = "flex items-center p-6 pt-0";

  const classes = `${baseClasses} ${className || ""}`;

  return <div className={classes} {...props} />;
};

export {
  Card,
  CardHeader,
  CardFooter,
  CardTitle,
  CardDescription,
  CardContent,
};