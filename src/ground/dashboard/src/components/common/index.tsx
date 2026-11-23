// Common reusable components

import React from 'react';
import { cn } from '@/utils/helpers';

// Button component
export interface ButtonProps extends React.ButtonHTMLAttributes<HTMLButtonElement> {
  variant?: 'primary' | 'secondary' | 'danger' | 'ghost' | 'outline';
  size?: 'sm' | 'md' | 'lg';
  loading?: boolean;
  icon?: React.ReactNode;
  iconPosition?: 'left' | 'right';
}

export const Button = React.forwardRef<HTMLButtonElement, ButtonProps>(
  ({ className, variant = 'primary', size = 'md', loading, icon, iconPosition = 'left', children, disabled, ...props }, ref) => {
    const baseStyles = 'inline-flex items-center justify-center font-medium rounded-lg transition-colors focus:outline-none focus:ring-2 focus:ring-offset-2 disabled:opacity-50 disabled:cursor-not-allowed';
    
    const variantStyles = {
      primary: 'bg-primary-600 text-white hover:bg-primary-700 focus:ring-primary-500',
      secondary: 'bg-gray-100 text-gray-900 hover:bg-gray-200 focus:ring-gray-500',
      danger: 'bg-danger-600 text-white hover:bg-danger-700 focus:ring-danger-500',
      ghost: 'bg-transparent text-gray-700 hover:bg-gray-100 focus:ring-gray-500',
      outline: 'border-2 border-gray-300 text-gray-700 hover:bg-gray-50 focus:ring-gray-500',
    };
    
    const sizeStyles = {
      sm: 'px-3 py-1.5 text-sm gap-1.5',
      md: 'px-4 py-2 text-base gap-2',
      lg: 'px-6 py-3 text-lg gap-2',
    };

    return (
      <button
        ref={ref}
        className={cn(baseStyles, variantStyles[variant], sizeStyles[size], className)}
        disabled={disabled || loading}
        {...props}
      >
        {loading && (
          <svg className="animate-spin h-4 w-4" viewBox="0 0 24 24">
            <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4" fill="none" />
            <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4z" />
          </svg>
        )}
        {icon && iconPosition === 'left' && !loading && <span>{icon}</span>}
        {children}
        {icon && iconPosition === 'right' && !loading && <span>{icon}</span>}
      </button>
    );
  }
);

Button.displayName = 'Button';

// Card component
export interface CardProps extends React.HTMLAttributes<HTMLDivElement> {
  variant?: 'default' | 'outlined' | 'elevated';
  padding?: 'none' | 'sm' | 'md' | 'lg';
}

export const Card = React.forwardRef<HTMLDivElement, CardProps>(
  ({ className, variant = 'default', padding = 'md', children, ...props }, ref) => {
    const variantStyles = {
      default: 'bg-white border border-gray-200',
      outlined: 'bg-white border-2 border-gray-300',
      elevated: 'bg-white shadow-lg border-none',
    };
    
    const paddingStyles = {
      none: '',
      sm: 'p-3',
      md: 'p-4',
      lg: 'p-6',
    };

    return (
      <div
        ref={ref}
        className={cn('rounded-xl', variantStyles[variant], paddingStyles[padding], className)}
        {...props}
      >
        {children}
      </div>
    );
  }
);

Card.displayName = 'Card';

// CardHeader, CardContent, CardFooter
export const CardHeader = React.forwardRef<HTMLDivElement, React.HTMLAttributes<HTMLDivElement>>(
  ({ className, children, ...props }, ref) => (
    <div ref={ref} className={cn('px-4 py-3 border-b border-gray-200', className)} {...props}>
      {children}
    </div>
  )
);

CardHeader.displayName = 'CardHeader';

export const CardContent = React.forwardRef<HTMLDivElement, React.HTMLAttributes<HTMLDivElement>>(
  ({ className, children, ...props }, ref) => (
    <div ref={ref} className={cn('p-4', className)} {...props}>
      {children}
    </div>
  )
);

CardContent.displayName = 'CardContent';

export const CardFooter = React.forwardRef<HTMLDivElement, React.HTMLAttributes<HTMLDivElement>>(
  ({ className, children, ...props }, ref) => (
    <div ref={ref} className={cn('px-4 py-3 border-t border-gray-200 bg-gray-50 rounded-b-xl', className)} {...props}>
      {children}
    </div>
  )
);

CardFooter.displayName = 'CardFooter';

// Badge component
export interface BadgeProps extends React.HTMLAttributes<HTMLSpanElement> {
  variant?: 'default' | 'success' | 'warning' | 'danger' | 'info' | 'outline';
  size?: 'sm' | 'md' | 'lg';
  dot?: boolean;
}

export const Badge = React.forwardRef<HTMLSpanElement, BadgeProps>(
  ({ className, variant = 'default', size = 'md', dot, children, ...props }, ref) => {
    const variantStyles = {
      default: 'bg-gray-100 text-gray-700',
      success: 'bg-success-100 text-success-700',
      warning: 'bg-warning-100 text-warning-700',
      danger: 'bg-danger-100 text-danger-700',
      info: 'bg-primary-100 text-primary-700',
      outline: 'border-2 border-gray-300 text-gray-700',
    };
    
    const sizeStyles = {
      sm: 'px-2 py-0.5 text-xs',
      md: 'px-2.5 py-1 text-sm',
      lg: 'px-3 py-1 text-base',
    };

    return (
      <span
        ref={ref}
        className={cn(
          'inline-flex items-center gap-1 font-medium rounded-full',
          variantStyles[variant],
          sizeStyles[size],
          className
        )}
        {...props}
      >
        {dot && <span className="w-1.5 h-1.5 rounded-full bg-current" />}
        {children}
      </span>
    )
  }
);

Badge.displayName = 'Badge';

// Input component
export interface InputProps extends React.InputHTMLAttributes<HTMLInputElement> {
  label?: string;
  error?: string;
  helperText?: string;
  leftIcon?: React.ReactNode;
  rightIcon?: React.ReactNode;
}

export const Input = React.forwardRef<HTMLInputElement, InputProps>(
  ({ className, label, error, helperText, leftIcon, rightIcon, id, ...props }, ref) => {
    const inputId = id || `input-${Math.random().toString(36).substr(2, 9)}`;
    const errorId = `${inputId}-error`;
    const helperId = `${inputId}-helper`;

    return (
      <div className="w-full">
        {label && (
          <label htmlFor={inputId} className="block text-sm font-medium text-gray-700 mb-1">
            {label}
          </label>
        )}
        <div className="relative">
          {leftIcon && (
            <div className="absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none text-gray-400">
              {leftIcon}
            </div>
          )}
          <input
            ref={ref}
            id={inputId}
            className={cn(
              'block w-full rounded-lg border transition-colors',
              'placeholder:text-gray-400',
              'focus:outline-none focus:ring-2 focus:ring-primary-500 focus:border-primary-500',
              'disabled:bg-gray-100 disabled:cursor-not-allowed',
              error
                ? 'border-danger-500 focus:ring-danger-500 focus:border-danger-500'
                : 'border-gray-300',
              leftIcon ? 'pl-10' : 'pl-4',
              rightIcon ? 'pr-10' : 'pr-4',
              'py-2'
            )}
            aria-invalid={error ? 'true' : 'false'}
            aria-describedby={error ? errorId : helperText ? helperId : undefined}
            {...props}
          />
          {rightIcon && (
            <div className="absolute inset-y-0 right-0 pr-3 flex items-center pointer-events-none text-gray-400">
              {rightIcon}
            </div>
          )}
        </div>
        {error && (
          <p id={errorId} className="mt-1 text-sm text-danger-600" role="alert">
            {error}
          </p>
        )}
        {helperText && !error && (
          <p id={helperId} className="mt-1 text-sm text-gray-500">
            {helperText}
          </p>
        )}
      </div>
    )
  }
);

Input.displayName = 'Input';

// Select component
export interface SelectProps extends React.SelectHTMLAttributes<HTMLSelectElement> {
  label?: string;
  error?: string;
  helperText?: string;
  options: { value: string; label: string }[];
  placeholder?: string;
}

export const Select = React.forwardRef<HTMLSelectElement, SelectProps>(
  ({ className, label, error, helperText, options, placeholder, id, ...props }, ref) => {
    const selectId = id || `select-${Math.random().toString(36).substr(2, 9)}`;
    const errorId = `${selectId}-error`;
    const helperId = `${selectId}-helper`;

    return (
      <div className="w-full">
        {label && (
          <label htmlFor={selectId} className="block text-sm font-medium text-gray-700 mb-1">
            {label}
          </label>
        )}
        <select
          ref={ref}
          id={selectId}
          className={cn(
            'block w-full rounded-lg border transition-colors py-2 pl-4 pr-10',
            'focus:outline-none focus:ring-2 focus:ring-primary-500 focus:border-primary-500',
            'disabled:bg-gray-100 disabled:cursor-not-allowed',
            error
              ? 'border-danger-500 focus:ring-danger-500 focus:border-danger-500'
              : 'border-gray-300',
            'appearance-none bg-white'
          )}
          aria-invalid={error ? 'true' : 'false'}
          aria-describedby={error ? errorId : helperText ? helperId : undefined}
          {...props}
        >
          {placeholder && (
            <option value="" disabled>
              {placeholder}
            </option>
          )}
          {options.map((option) => (
            <option key={option.value} value={option.value}>
              {option.label}
            </option>
          ))}
        </select>
        {error && (
          <p id={errorId} className="mt-1 text-sm text-danger-600" role="alert">
            {error}
          </p>
        )}
        {helperText && !error && (
          <p id={helperId} className="mt-1 text-sm text-gray-500">
            {helperText}
          </p>
        )}
      </div>
    )
  }
);

Select.displayName = 'Select';

// Modal component
export interface ModalProps {
  isOpen: boolean;
  onClose: () => void;
  title?: string;
  children: React.ReactNode;
  size?: 'sm' | 'md' | 'lg' | 'xl' | 'full';
  showCloseButton?: boolean;
}

export const Modal: React.FC<ModalProps> = ({
  isOpen,
  onClose,
  title,
  children,
  size = 'md',
  showCloseButton = true,
}) => {
  if (!isOpen) return null;

  const sizeStyles = {
    sm: 'max-w-sm',
    md: 'max-w-md',
    lg: 'max-w-lg',
    xl: 'max-w-xl',
    full: 'max-w-4xl',
  };

  return (
    <div className="fixed inset-0 z-50 overflow-y-auto">
      <div className="flex min-h-full items-center justify-center p-4">
        <div 
          className="fixed inset-0 bg-black/50 transition-opacity" 
          onClick={onClose}
          aria-hidden="true"
        />
        <div className={cn('relative w-full bg-white rounded-xl shadow-xl transform transition-all', sizeStyles[size])}>
          {(title || showCloseButton) && (
            <div className="flex items-center justify-between p-4 border-b border-gray-200">
              {title && <h2 className="text-lg font-semibold text-gray-900">{title}</h2>}
              {showCloseButton && (
                <button
                  onClick={onClose}
                  className="p-1 text-gray-400 hover:text-gray-600 rounded-lg hover:bg-gray-100 transition-colors"
                  aria-label="Close modal"
                >
                  <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
                  </svg>
                </button>
              )}
            </div>
          )}
          <div className="p-4">
            {children}
          </div>
        </div>
      </div>
    </div>
  );
};

// Tooltip component
export interface TooltipProps {
  content: React.ReactNode;
  children: React.ReactElement;
  position?: 'top' | 'bottom' | 'left' | 'right';
  delay?: number;
}

export const Tooltip: React.FC<TooltipProps> = ({
  content,
  children,
  position = 'top',
  delay = 200,
}) => {
  const [isVisible, setIsVisible] = React.useState(false);
  const timeoutRef = React.useRef<NodeJS.Timeout | null>(null);
  const childRef = React.useRef<HTMLElement>(null);

  const show = () => {
    if (timeoutRef.current) clearTimeout(timeoutRef.current);
    timeoutRef.current = setTimeout(() => setIsVisible(true), delay);
  };

  const hide = () => {
    if (timeoutRef.current) clearTimeout(timeoutRef.current);
    setIsVisible(false);
  };

  React.useEffect(() => {
    const el = childRef.current;
    if (!el) return;
    el.addEventListener('mouseenter', show);
    el.addEventListener('mouseleave', hide);
    el.addEventListener('focus', show);
    el.addEventListener('blur', hide);
    return () => {
      el.removeEventListener('mouseenter', show);
      el.removeEventListener('mouseleave', hide);
      el.removeEventListener('focus', show);
      el.removeEventListener('blur', hide);
    };
  }, []);

  const positionStyles = {
    top: 'bottom-full left-1/2 -translate-x-1/2 mb-2',
    bottom: 'top-full left-1/2 -translate-x-1/2 mt-2',
    left: 'right-full top-1/2 -translate-y-1/2 mr-2',
    right: 'left-full top-1/2 -translate-y-1/2 ml-2',
  };

  const arrowStyles = {
    top: 'top-full left-1/2 -translate-x-1/2 border-t-gray-900',
    bottom: 'bottom-full left-1/2 -translate-x-1/2 border-b-gray-900',
    left: 'left-full top-1/2 -translate-y-1/2 border-l-gray-900',
    right: 'right-full top-1/2 -translate-y-1/2 border-r-gray-900',
  };

  return (
    <span ref={childRef} onMouseEnter={show} onMouseLeave={hide} onFocus={show} onBlur={hide}>
      {children}
      {isVisible && (
        <div className={cn('fixed z-50 px-2 py-1 text-xs font-medium text-white bg-gray-900 rounded shadow-lg whitespace-nowrap', positionStyles[position])}>
          {content}
          <div className={cn('absolute w-0 h-0 border-4 border-transparent', arrowStyles[position])} />
        </div>
      )}
    </span>
  );
};

// Spinner component
export interface SpinnerProps {
  size?: 'sm' | 'md' | 'lg';
  className?: string;
}

export const Spinner: React.FC<SpinnerProps> = ({ size = 'md', className }) => {
  const sizeStyles = {
    sm: 'w-4 h-4 border-2',
    md: 'w-6 h-6 border-3',
    lg: 'w-10 h-10 border-4',
  };

  return (
    <svg className={cn('animate-spin text-primary-600', sizeStyles[size], className)} viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
      <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4" fill="none" />
      <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4z" />
    </svg>
  );
};

// Divider component
export const Divider: React.FC<{ className?: string; orientation?: 'horizontal' | 'vertical' }> = ({ 
  className, 
  orientation = 'horizontal' 
}) => (
  <div className={cn(
    'bg-gray-200',
    orientation === 'horizontal' ? 'h-px w-full' : 'w-px h-full',
    className
  )} role="separator" />
);

// Avatar component
export interface AvatarProps {
  src?: string;
  alt?: string;
  name?: string;
  size?: 'xs' | 'sm' | 'md' | 'lg' | 'xl';
  status?: 'online' | 'offline' | 'busy' | 'away';
  className?: string;
}

export const Avatar: React.FC<AvatarProps> = ({
  src,
  alt,
  name,
  size = 'md',
  status,
  className,
}) => {
  const sizeStyles = {
    xs: 'w-6 h-6 text-xs',
    sm: 'w-8 h-8 text-sm',
    md: 'w-10 h-10 text-base',
    lg: 'w-12 h-12 text-lg',
    xl: 'w-16 h-16 text-xl',
  };

  const statusStyles = {
    online: 'bg-success-500',
    offline: 'bg-gray-400',
    busy: 'bg-danger-500',
    away: 'bg-warning-500',
  };

  const statusSizeStyles = {
    xs: 'w-1.5 h-1.5',
    sm: 'w-2 h-2',
    md: 'w-2.5 h-2.5',
    lg: 'w-3 h-3',
    xl: 'w-4 h-4',
  };

  const initials = name
    ? name.split(' ').map(n => n[0]).join('').toUpperCase().slice(0, 2)
    : '?';

  return (
    <div className={cn('relative inline-flex', className)}>
      {src ? (
        <img
          src={src}
          alt={alt || name || 'Avatar'}
          className={cn('rounded-full object-cover', sizeStyles[size])}
        />
      ) : (
        <div className={cn('rounded-full bg-primary-100 flex items-center justify-center text-primary-700 font-medium', sizeStyles[size])}>
          {initials}
        </div>
      )}
      {status && (
        <span className={cn(
          'absolute bottom-0 right-0 rounded-full border-2 border-white',
          statusStyles[status],
          statusSizeStyles[size]
        )} />
      )}
    </div>
  );
);

// Table components
export interface TableProps<T> {
  data: T[];
  columns: Column<T>[];
  keyExtractor: (item: T) => string;
  onRowClick?: (item: T) => void;
  selectable?: boolean;
  selectedKeys?: string[];
  onSelectionChange?: (keys: string[]) => void;
  loading?: boolean;
  emptyMessage?: string;
}

export interface Column<T> {
  key: string;
  header: string;
  render?: (item: T) => React.ReactNode;
  width?: string;
  align?: 'left' | 'center' | 'right';
  sortable?: boolean;
}

export function Table<T>({ 
  data, 
  columns, 
  keyExtractor, 
  onRowClick,
  selectable,
  selectedKeys = [],
  onSelectionChange,
  loading,
  emptyMessage = 'No data available'
}: TableProps<T>) {
  const [sortConfig, setSortConfig] = React.useState<{ key: string; direction: 'asc' | 'desc' } | null>(null);
  const [selection, setSelection] = React.useState<Set<string>>(new Set(selectedKeys));

  const handleSort = (key: string) => {
    const column = columns.find(c => c.key === key);
    if (!column?.sortable) return;
    
    setSortConfig(current => ({
      key,
      direction: current?.key === key && current.direction === 'asc' ? 'desc' : 'asc',
    }));
  };

  const sortedData = React.useMemo(() => {
    if (!sortConfig) return data;
    
    return [...data].sort((a, b) => {
      const aVal = a[sortConfig.key as keyof T];
      const bVal = b[sortConfig.key as keyof T];
      
      if (aVal < bVal) return sortConfig.direction === 'asc' ? -1 : 1;
      if (aVal > bVal) return sortConfig.direction === 'asc' ? 1 : -1;
      return 0;
    });
  }, [data, sortConfig]);

  const handleSelectionChange = (key: string, checked: boolean) => {
    const newSelection = new Set(selection);
    if (checked) newSelection.add(key);
    else newSelection.delete(key);
    setSelection(newSelection);
    onSelectionChange?.(Array.from(newSelection));
  };

  const handleRowSelectionChange = (key: string) => {
    const checked = !selection.has(key);
    handleSelectionChange(key, checked);
  };

  if (loading) {
    return (
      <div className="w-full py-8 text-center">
        <Spinner />
        <p className="mt-2 text-gray-500">Loading...</p>
      </div>
    );
  }

  if (data.length === 0) {
    return (
      <div className="w-full py-8 text-center text-gray-500">
        {emptyMessage}
      </div>
    );
  }

  return (
    <div className="w-full overflow-x-auto">
      <table className="w-full text-sm">
        <thead className="bg-gray-50">
          <tr>
            {selectable && (
              <th className="px-4 py-3 text-left">
                <input
                  type="checkbox"
                  checked={selection.size === data.length && data.length > 0}
                  indeterminate={selection.size > 0 && selection.size < data.length}
                  onChange={(e) => {
                    if (e.target.checked) {
                      setSelection(new Set(data.map(keyExtractor)));
                    } else {
                      setSelection(new Set());
                    }
                    onSelectionChange?.(e.target.checked ? data.map(keyExtractor) : []);
                  }}
                  className="rounded border-gray-300 text-primary-600 focus:ring-primary-500"
                />
              </th>
            )}
            {columns.map((column) => (
              <th
                key={column.key}
                className={cn(
                  'px-4 py-3 text-left text-xs font-semibold text-gray-500 uppercase tracking-wider',
                  column.sortable && 'cursor-pointer hover:bg-gray-100 select-none'
                )}
                style={{ width: column.width }}
                onClick={() => column.sortable && handleSort(column.key)}
              >
                <div className="flex items-center gap-1">
                  {column.header}
                  {column.sortable && (
                    <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M7 16l4-4m0 0l4 4m-4-4v12" />
                    </svg>
                  )}
                </div>
              </th>
            ))}
          </tr>
        </thead>
        <tbody className="divide-y divide-gray-200 bg-white">
          {sortedData.map((item) => {
            const key = keyExtractor(item);
            const isSelected = selection.has(key);
            
            return (
              <tr
                key={key}
                className={cn(
                  'transition-colors',
                  isSelected && 'bg-primary-50',
                  onRowClick && 'cursor-pointer hover:bg-gray-50'
                )}
                onClick={() => onRowClick?.(item)}
              >
                {selectable && (
                  <td className="px-4 py-3">
                    <input
                      type="checkbox"
                      checked={isSelected}
                      onChange={(e) => handleRowSelectionChange(key)}
                      className="rounded border-gray-300 text-primary-600 focus:ring-primary-500"
                    />
                  </td>
                )}
                {columns.map((column) => (
                  <td
                    key={column.key}
                    className={cn(
                      'px-4 py-3',
                      column.align === 'center' && 'text-center',
                      column.align === 'right' && 'text-right'
                    )}
                    style={{ width: column.width }}
                  >
                    {column.render 
                      ? column.render(item) 
                      : String(item[column.key as keyof T] ?? '')}
                  </td>
                ))}
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}

// Export all common components
export const CommonComponents = {
  Button,
  Card,
  CardHeader,
  CardContent,
  CardFooter,
  Badge,
  Input,
  Select,
  Modal,
  Tooltip,
  Spinner,
  Divider,
  Avatar,
  Table,
};