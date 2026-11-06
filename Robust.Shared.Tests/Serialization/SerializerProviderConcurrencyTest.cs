using System;
using System.Linq;
using System.Threading.Tasks;
using NUnit.Framework;
using Robust.Shared.IoC;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager;
using Robust.Shared.Serialization.Markdown;
using Robust.Shared.Serialization.Markdown.Value;
using Robust.Shared.Serialization.TypeSerializers.Interfaces;

namespace Robust.Shared.Tests.Serialization;

[TestFixture]
public sealed class SerializerProviderConcurrencyTest
{
    [Test]
    public void ConcurrentRegistrationsPreserveEveryIndexedCopierAndWriter()
    {
        var arguments = new[] { typeof(int), typeof(string), typeof(bool), typeof(byte), typeof(long), typeof(float),
            typeof(double), typeof(decimal), typeof(Guid), typeof(DateTime), typeof(short), typeof(uint),
            typeof(ulong), typeof(ushort), typeof(char), typeof(TimeSpan) };
        var types = arguments.Concat(arguments.SelectMany(a => arguments.Select(b => typeof(Pair<,>).MakeGenericType(a, b)))).ToArray();
        var registrations = types.Select(type => (IRegistration) Activator.CreateInstance(typeof(Registration<>).MakeGenericType(type))!).ToArray();
        var manager = new SerializationManager();
        for (var round = 0; round < 128; round++)
        {
            // Each fresh provider starts with an empty indexed cache, while type
            // IDs are process-wide. Different registrations must never publish a
            // smaller replacement array over another thread's larger one.
            var provider = new SerializationManager.SerializerProvider(manager);
            Parallel.For(0, registrations.Length, new ParallelOptions { MaxDegreeOfParallelism = 8 }, i =>
            {
                var registration = registrations[(i * 131 + round) % registrations.Length];
                registration.Register(provider);
                registration.Verify(provider);
            });
            foreach (var registration in registrations) registration.Verify(provider);
        }
    }

    private sealed class Pair<TFirst, TSecond>;
    private interface IRegistration
    {
        void Register(SerializationManager.SerializerProvider provider);
        void Verify(SerializationManager.SerializerProvider provider);
    }
    private sealed class Registration<T> : IRegistration, ITypeCopyCreator<T>, ITypeWriter<T>
    {
        public Registration() { }
        public void Register(SerializationManager.SerializerProvider provider) => provider.RegisterSerializer(this);
        public void Verify(SerializationManager.SerializerProvider provider)
        {
            Assert.That(provider.TryGetCopierOrCreator<T>(out _, out var creator), Is.True);
            Assert.That(creator, Is.SameAs(this));
            Assert.That(provider.TryGetTypeSerializer(typeof(ITypeWriter<>), typeof(T), out var writer), Is.True);
            Assert.That(writer, Is.SameAs(this));
        }
        public T CreateCopy(ISerializationManager serializationManager, T source, IDependencyCollection dependencies,
            SerializationHookContext hookCtx, ISerializationContext? context = null) => source;
        public DataNode Write(ISerializationManager serializationManager, T value, IDependencyCollection dependencies,
            bool alwaysWrite = false, ISerializationContext? context = null) => new ValueDataNode("unused");
    }
}
