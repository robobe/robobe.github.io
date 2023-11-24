---
tags:
    - microsoft
    - dotnet
    - .net
    - c#
    - vscode
---

# Install .NET on ubuntu 22.04

## Install
```bash title=".NET SDK"
sudo apt install dotnet-sdk-7.0
```

## Template

```bash
dotnet new console --framework net7.0
```

## Hello
- Replace `Program.cs`

```csharp
namespace HelloWorld
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Hello, World!");
        }
    }
}
```

!!! tip "missing asset"
     Select 'yes'  when vscode ask about missing asset
     Add tasks.json and launch.json files

     Add files manually

     - Select Run > Add Configuration
     - Select `.NET5 + and .NET Core`

!!! tip
    `f5` build and run in debug

---

## Reference
- [Tutorial: Create a .NET console application using Visual Studio Code](https://learn.microsoft.com/en-us/dotnet/core/tutorials/with-visual-studio-code?pivots=dotnet-7-0)
- [How to: Raise and Consume Events](https://learn.microsoft.com/en-us/dotnet/standard/events/how-to-raise-and-consume-events)
- [Handle and raise events](https://learn.microsoft.com/en-us/dotnet/standard/events/)